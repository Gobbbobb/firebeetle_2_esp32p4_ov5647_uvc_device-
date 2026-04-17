/*
 * FireBeetle ESP32-P4 — UVC device v3.1
 *
 * ONE FIX FROM v3
 * ───────────────
 * [FIX-CACHE] Add esp_cache_msync(out_buf, C2M) between PPA output and
 *             JPEG encoder input.
 *
 * Root cause of truncated frames (8–15 KB instead of expected 30–80 KB):
 *
 *   PPA SRM writes to out_buf via AXI-DMA.  On ESP32-P4, AXI-DMA writes
 *   go into the L2 cache but are NOT guaranteed to be flushed to physical
 *   PSRAM before the transaction completes.
 *
 *   The HW JPEG encoder reads out_buf via a separate DMA path that reads
 *   directly from physical PSRAM, bypassing L2 cache.  Without a C2M
 *   flush, the encoder reads stale / zeroed data from PSRAM → produces a
 *   valid-header JPEG of mostly zeros → extremely small compressed output
 *   (~15 KB).  The host decoder then hits unexpected-EOF error 6.
 *
 *   Fix: after ppa_do_scale_rotate_mirror() returns (blocking, DMA done),
 *   flush L2 cache → PSRAM for the out_buf region before queuing the
 *   encoder input buffer.
 *
 * Everything else is identical to v3.
 */

#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "driver/ppa.h"
#include "usb_device_uvc.h"
#include "uvc_frame_config.h"
#include "example_video_common.h"
#include "esp_check.h"

/* ── device nodes ─────────────────────────────────────────────────── */
#define CAM_DEV_PATH  "/dev/video0"
#define ENC_DEV_PATH  "/dev/video10"

/* ── geometry ─────────────────────────────────────────────────────── */
#define CAM_W  1280
#define CAM_H   960
#define OUT_W   480     /* 90°CW-rotated output width  */
#define OUT_H   640     /* 90°CW-rotated output height */

/* ── buffer constants ─────────────────────────────────────────────── */
#define MEMORY_TYPE    V4L2_MEMORY_USERPTR
#define MEMORY_ALIGN   64
#define CAM_BUF_COUNT  4

#define RGB_SIZE      (CAM_W * CAM_H * 2)
#define OUT_RGB_SIZE  (OUT_W * OUT_H * 2)
#define JPEG_BUF_SIZE  OUT_RGB_SIZE

/* Round up to 64-byte cache line (required by esp_cache_msync) */
#define ALIGN64(x)  (((size_t)(x) + 63u) & ~(size_t)63u)

#define JPEG_QUALITY  85

static const char *TAG = "uvc_dev_v31";

typedef struct {
    int      cam_fd;
    uint8_t *cam_buf[CAM_BUF_COUNT];
    uint32_t cam_buf_len[CAM_BUF_COUNT];

    ppa_client_handle_t ppa;
    uint8_t *out_buf;              /* PPA output: OUT_W × OUT_H RGB565 */

    int      enc_fd;
    uint8_t *jpeg_buf[2];
    uint32_t jpeg_len[2];
    int      jpeg_ready_idx;

    SemaphoreHandle_t frame_ready;
    SemaphoreHandle_t frame_done;

    uvc_fb_t  fb;
    uint32_t  frame_count;
} ctx_t;

/* ── PPA: 1280×960 → 480×640 (scale 0.5× + rotate 90°CW) ─────────── */
static esp_err_t ppa_downscale_rot90(ctx_t *ctx,
                                      const uint8_t *src,
                                      uint8_t       *dst)
{
    ppa_srm_oper_config_t cfg = {
        .in = {
            .buffer         = (void *)src,
            .pic_w          = CAM_W,
            .pic_h          = CAM_H,
            .block_w        = CAM_W,
            .block_h        = CAM_H,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer         = (void *)dst,
            .buffer_size    = OUT_RGB_SIZE,
            .pic_w          = OUT_W,
            .pic_h          = OUT_H,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_90,
        .scale_x        = 0.5f,
        .scale_y        = 0.5f,
        .mirror_x       = false,
        .mirror_y       = false,
        .mode           = PPA_TRANS_MODE_BLOCKING,
    };
    return ppa_do_scale_rotate_mirror(ctx->ppa, &cfg);
}

static void jpeg_set_quality(int enc_fd, int q)
{
    struct v4l2_jpegcompression jc = {0};
    if (ioctl(enc_fd, VIDIOC_G_JPEGCOMP, &jc) == 0) {
        jc.quality = q;
        if (ioctl(enc_fd, VIDIOC_S_JPEGCOMP, &jc) == 0)
            ESP_LOGI(TAG, "JPEG quality=%d", q);
    }
}

/* ── Pipeline task (Core 1) ───────────────────────────────────────── */
static void pipeline_task(void *arg)
{
    ctx_t *ctx = (ctx_t *)arg;
    int out_idx = 0;

    for (;;) {
        struct v4l2_buffer vbuf = {
            .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = MEMORY_TYPE,
        };
        if (ioctl(ctx->cam_fd, VIDIOC_DQBUF, &vbuf) != 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        uint8_t *rgb_in = ctx->cam_buf[vbuf.index];

        /* Invalidate CPU cache for ISP buffer: ISP DMA wrote to PSRAM */
        esp_cache_msync(rgb_in, ALIGN64(ctx->cam_buf_len[vbuf.index]),
                        ESP_CACHE_MSYNC_FLAG_INVALIDATE);

        /* PPA: scale 0.5× + rotate 90°CW (blocking) */
        esp_err_t r = ppa_downscale_rot90(ctx, rgb_in, ctx->out_buf);
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "PPA err=0x%x", r);
            goto requeue;
        }

        /*
         * [FIX-CACHE] Flush PPA output from L2 cache → physical PSRAM.
         *
         * PPA AXI-DMA writes land in L2 cache but may not reach PSRAM
         * before this function returns.  The HW JPEG encoder's DMA reads
         * from physical PSRAM directly.  Without this flush the encoder
         * sees stale/zeroed data → produces a tiny (~15KB) near-empty JPEG
         * → host decoder hits error 6 (unexpected EOF).
         */
        esp_cache_msync(ctx->out_buf, ALIGN64(OUT_RGB_SIZE),
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M);

        /* JPEG encode: out_buf (480×640 RGB565) → jpeg_buf[out_idx] */
        {
            struct v4l2_buffer enc_out = {
                .type      = V4L2_BUF_TYPE_VIDEO_OUTPUT,
                .memory    = V4L2_MEMORY_USERPTR, .index = 0,
                .m.userptr = (unsigned long)ctx->out_buf,
                .length    = OUT_RGB_SIZE,
                .bytesused = OUT_RGB_SIZE,
            };
            struct v4l2_buffer enc_cap = {
                .type      = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                .memory    = V4L2_MEMORY_USERPTR, .index = 0,
                .m.userptr = (unsigned long)ctx->jpeg_buf[out_idx],
                .length    = JPEG_BUF_SIZE,
            };

            bool oq = (ioctl(ctx->enc_fd, VIDIOC_QBUF, &enc_out) == 0);
            bool cq = oq && (ioctl(ctx->enc_fd, VIDIOC_QBUF, &enc_cap) == 0);

            if (cq && ioctl(ctx->enc_fd, VIDIOC_DQBUF, &enc_cap) == 0) {
                ioctl(ctx->enc_fd, VIDIOC_DQBUF, &enc_out);
                ctx->jpeg_len[out_idx] = enc_cap.bytesused;
                ctx->frame_count++;
                if (ctx->frame_count % 45 == 0)
                    ESP_LOGI(TAG, "frame=%lu  jpeg=%lu B",
                             (unsigned long)ctx->frame_count,
                             (unsigned long)enc_cap.bytesused);
            } else {
                if (cq) ioctl(ctx->enc_fd, VIDIOC_DQBUF, &enc_cap);
                if (oq) ioctl(ctx->enc_fd, VIDIOC_DQBUF, &enc_out);
                ESP_LOGW(TAG, "Encoder fail errno=%d", errno);
                ctx->jpeg_len[out_idx] = 0;
            }
        }

requeue:
        vbuf.m.userptr = (unsigned long)ctx->cam_buf[vbuf.index];
        vbuf.length    = ctx->cam_buf_len[vbuf.index];
        vbuf.bytesused = 0;
        ioctl(ctx->cam_fd, VIDIOC_QBUF, &vbuf);

        if (ctx->jpeg_len[out_idx] > 0) {
            ctx->jpeg_ready_idx = out_idx;
            xSemaphoreGive(ctx->frame_ready);
            xSemaphoreTake(ctx->frame_done, portMAX_DELAY);
            out_idx ^= 1;
        }
    }
}

/* ── UVC callbacks ────────────────────────────────────────────────── */
static esp_err_t video_start_cb(uvc_format_t fmt, int w, int h, int fps,
                                 void *arg)
{
    ctx_t *ctx = (ctx_t *)arg;
    ESP_LOGI(TAG, "Host start %dx%d@%d", w, h, fps);
    for (int i = 0; i < CAM_BUF_COUNT; i++) {
        struct v4l2_buffer q = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = MEMORY_TYPE,
            .index = i, .m.userptr = (unsigned long)ctx->cam_buf[i],
            .length = ctx->cam_buf_len[i],
        };
        ioctl(ctx->cam_fd, VIDIOC_QBUF, &q);
    }
    int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(ctx->cam_fd, VIDIOC_STREAMON, &t);
    ctx->frame_count = 0;
    return ESP_OK;
}

static void video_stop_cb(void *arg)
{
    ctx_t *ctx = (ctx_t *)arg;
    int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(ctx->cam_fd, VIDIOC_STREAMOFF, &t);
    ESP_LOGI(TAG, "Stop after %lu frames", (unsigned long)ctx->frame_count);
}

static uvc_fb_t *video_fb_get_cb(void *arg)
{
    ctx_t *ctx = (ctx_t *)arg;
    if (xSemaphoreTake(ctx->frame_ready, pdMS_TO_TICKS(120)) != pdTRUE)
        return NULL;
    int idx = ctx->jpeg_ready_idx;
    int64_t ts = esp_timer_get_time();
    ctx->fb.buf               = ctx->jpeg_buf[idx];
    ctx->fb.len               = ctx->jpeg_len[idx];
    ctx->fb.width             = OUT_W;
    ctx->fb.height            = OUT_H;
    ctx->fb.format            = UVC_FORMAT_JPEG;
    ctx->fb.timestamp.tv_sec  = (long)(ts / 1000000UL);
    ctx->fb.timestamp.tv_usec = (long)(ts % 1000000UL);
    return &ctx->fb;
}

static void video_fb_return_cb(uvc_fb_t *fb, void *arg)
{
    xSemaphoreGive(((ctx_t *)arg)->frame_done);
}

/* ── Pipeline init ────────────────────────────────────────────────── */
static esp_err_t pipeline_init(ctx_t *ctx)
{
    ctx->frame_ready = xSemaphoreCreateBinary();
    ctx->frame_done  = xSemaphoreCreateBinary();
    if (!ctx->frame_ready || !ctx->frame_done) return ESP_ERR_NO_MEM;
    xSemaphoreGive(ctx->frame_done);

    /* PPA SRM client */
    {
        ppa_client_config_t c = { .oper_type = PPA_OPERATION_SRM };
        ESP_RETURN_ON_ERROR(ppa_register_client(&c, &ctx->ppa), TAG, "PPA");
    }

    /* Buffers */
    ctx->out_buf = heap_caps_aligned_alloc(MEMORY_ALIGN, OUT_RGB_SIZE,
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (!ctx->out_buf) return ESP_ERR_NO_MEM;

    for (int i = 0; i < 2; i++) {
        ctx->jpeg_buf[i] = heap_caps_aligned_alloc(MEMORY_ALIGN, JPEG_BUF_SIZE,
                                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        if (!ctx->jpeg_buf[i]) return ESP_ERR_NO_MEM;
        esp_cache_msync(ctx->jpeg_buf[i], ALIGN64(JPEG_BUF_SIZE),
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    }

    /* Camera (ISP) */
    ctx->cam_fd = open(CAM_DEV_PATH, O_RDWR);
    if (ctx->cam_fd < 0) return ESP_FAIL;

    struct v4l2_format fmt_cam = {
        .type    = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix = { .width=CAM_W, .height=CAM_H, .pixelformat=V4L2_PIX_FMT_RGB565 },
    };
    if (ioctl(ctx->cam_fd, VIDIOC_S_FMT, &fmt_cam) != 0) return ESP_FAIL;

    struct v4l2_requestbuffers rb_cam = {
        .count=CAM_BUF_COUNT, .type=V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory=MEMORY_TYPE,
    };
    if (ioctl(ctx->cam_fd, VIDIOC_REQBUFS, &rb_cam) != 0) return ESP_FAIL;

    for (int i = 0; i < CAM_BUF_COUNT; i++) {
        struct v4l2_buffer q = {
            .type=V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory=MEMORY_TYPE, .index=i,
        };
        if (ioctl(ctx->cam_fd, VIDIOC_QUERYBUF, &q) != 0) return ESP_FAIL;
        ctx->cam_buf[i] = heap_caps_aligned_alloc(MEMORY_ALIGN, q.length,
                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        if (!ctx->cam_buf[i]) return ESP_ERR_NO_MEM;
        ctx->cam_buf_len[i] = q.length;
        esp_cache_msync(ctx->cam_buf[i], ALIGN64(q.length),
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        q.m.userptr = (unsigned long)ctx->cam_buf[i];
        if (ioctl(ctx->cam_fd, VIDIOC_QBUF, &q) != 0) return ESP_FAIL;
    }
    { int t = V4L2_BUF_TYPE_VIDEO_CAPTURE; ioctl(ctx->cam_fd, VIDIOC_STREAMON, &t); }
    ESP_LOGI(TAG, "Camera: %d bufs × %lu B", CAM_BUF_COUNT, (unsigned long)ctx->cam_buf_len[0]);

    /* JPEG encoder */
    ctx->enc_fd = open(ENC_DEV_PATH, O_RDWR);
    if (ctx->enc_fd < 0) return ESP_FAIL;

    struct v4l2_format fmt_out = {
        .type    = V4L2_BUF_TYPE_VIDEO_OUTPUT,
        .fmt.pix = { .width=OUT_W, .height=OUT_H, .pixelformat=V4L2_PIX_FMT_RGB565 },
    };
    if (ioctl(ctx->enc_fd, VIDIOC_S_FMT, &fmt_out) != 0) return ESP_FAIL;

    struct v4l2_format fmt_cap = {
        .type    = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix = { .width=OUT_W, .height=OUT_H, .pixelformat=V4L2_PIX_FMT_JPEG },
    };
    if (ioctl(ctx->enc_fd, VIDIOC_S_FMT, &fmt_cap) != 0) return ESP_FAIL;

    { struct v4l2_requestbuffers r={.count=1,.type=V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory=MEMORY_TYPE}; ioctl(ctx->enc_fd,VIDIOC_REQBUFS,&r); }
    { struct v4l2_requestbuffers r={.count=1,.type=V4L2_BUF_TYPE_VIDEO_CAPTURE,.memory=MEMORY_TYPE}; ioctl(ctx->enc_fd,VIDIOC_REQBUFS,&r); }
    { int t=V4L2_BUF_TYPE_VIDEO_OUTPUT;  ioctl(ctx->enc_fd,VIDIOC_STREAMON,&t); }
    { int t=V4L2_BUF_TYPE_VIDEO_CAPTURE; ioctl(ctx->enc_fd,VIDIOC_STREAMON,&t); }
    jpeg_set_quality(ctx->enc_fd, JPEG_QUALITY);

    /* UVC device */
    void *uvc_buf = heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!uvc_buf) return ESP_ERR_NO_MEM;
    uvc_device_config_t uvc_cfg = {
        .start_cb        = video_start_cb,
        .fb_get_cb       = video_fb_get_cb,
        .fb_return_cb    = video_fb_return_cb,
        .stop_cb         = video_stop_cb,
        .cb_ctx          = (void *)ctx,
        .uvc_buffer      = uvc_buf,
        .uvc_buffer_size = JPEG_BUF_SIZE,
    };
    ESP_ERROR_CHECK(uvc_device_config(0, &uvc_cfg));
    ESP_ERROR_CHECK(uvc_device_init());
    ESP_LOGI(TAG, "UVC ready: %dx%d MJPEG bulk 512B", OUT_W, OUT_H);

    xTaskCreatePinnedToCore(pipeline_task, "pipe", 4096, ctx, 5, NULL, 1);
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "v3.1  %dx%d→PPA→%dx%d→JPEG q%d→UVC",
             CAM_W, CAM_H, OUT_W, OUT_H, JPEG_QUALITY);
    vTaskDelay(pdMS_TO_TICKS(500));

    if (example_video_init() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed"); return;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    ctx_t *ctx = calloc(1, sizeof(ctx_t));
    if (!ctx || pipeline_init(ctx) != ESP_OK) {
        ESP_LOGE(TAG, "Pipeline init failed"); return;
    }
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "frames=%lu", (unsigned long)ctx->frame_count);
    }
}