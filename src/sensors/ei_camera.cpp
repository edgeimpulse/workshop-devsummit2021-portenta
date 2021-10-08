/* Edge Impulse ingestion SDK
 * Copyright (c) 2020 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ei_camera.h"
#include "ei_main.h"
#ifdef EI_CAMERA_FRAME_BUFFER_SDRAM
#include "SDRAM.h"
#endif

#define ALIGN_PTR(p,a)   ((p & (a-1)) ?(((uintptr_t)p + a) & ~(uintptr_t)(a-1)) : p)
#define DWORD_ALIGN_PTR(p)  ALIGN_PTR(p,4)

static CameraClass cam;
static bool is_initialised = false;
static bool is_ll_initialised = false;

/*
** @brief used to store the raw frame
*/
#if defined(EI_CAMERA_FRAME_BUFFER_SDRAM) || defined(EI_CAMERA_FRAME_BUFFER_HEAP)
static uint8_t *ei_camera_frame_mem;
static uint8_t *ei_camera_frame_buffer; // 32-byte aligned
#else
static uint8_t ei_camera_frame_buffer[EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS] __attribute__((aligned(32)));
#endif

/*
** @brief points to the output of the capture
*/
static uint8_t *ei_camera_capture_out = NULL;

static bool prepare_snapshot(size_t width, size_t height, bool use_max_baudrate);
static bool take_snapshot(size_t width, size_t height, bool print_oks);
static void finish_snapshot();

void resizeImage(int srcWidth, int srcHeight, uint8_t *srcImage, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp);
void cropImage(int srcWidth, int srcHeight, uint8_t *srcImage, int startX, int startY, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp);

/**
 * @brief      Convert monochrome data to rgb values
 *
 * @param[in]  mono_data  The mono data
 * @param      r          red pixel value
 * @param      g          green pixel value
 * @param      b          blue pixel value
 */
static inline void mono_to_rgb(uint8_t mono_data, uint8_t *r, uint8_t *g, uint8_t *b) {
    uint8_t v = mono_data;
    *r = *g = *b = v;
}

/**
 * @brief      Determine whether to resize and to which dimension
 *
 * @param[in]  out_width     width of output image
 * @param[in]  out_height    height of output image
 * @param[out] resize_col_sz       pointer to frame buffer's column/width value
 * @param[out] resize_row_sz       pointer to frame buffer's rows/height value
 * @param[out] do_resize     returns whether to resize (or not)
 *
 */
static int calculate_resize_dimensions(uint32_t out_width, uint32_t out_height, uint32_t *resize_col_sz, uint32_t *resize_row_sz, bool *do_resize)
{
    const ei_device_resize_resolutions_t *list;
    size_t list_size;

    int dl = EiDevice.get_resize_list((const ei_device_resize_resolutions_t **)&list, &list_size);
    if (dl) { /* apparently false is OK here?! */
        ei_printf("ERR: Device has no image resize feature\n");
        return 1;
    }

    // (default) conditions
    *resize_col_sz = EI_CAMERA_RAW_FRAME_BUFFER_COLS;
    *resize_row_sz = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
    *do_resize = false;

    for (size_t ix = 0; ix < list_size; ix++) {
        if ((out_width <= list[ix].width) && (out_height <= list[ix].height)) {
            *resize_col_sz = list[ix].width;
            *resize_row_sz = list[ix].height;
            *do_resize = true;
            break;
        }
    }

    return 0;
}

#include "mbed.h"
 static void print_memory_info2() {
     // allocate enough room for every thread's stack statistics
     int cnt = osThreadGetCount();
     mbed_stats_stack_t *stats = (mbed_stats_stack_t*) ei_malloc(cnt * sizeof(mbed_stats_stack_t));

     cnt = mbed_stats_stack_get_each(stats, cnt);
     for (int i = 0; i < cnt; i++) {
         ei_printf("Thread: 0x%lX, Stack size: %lu / %lu\r\n", stats[i].thread_id, stats[i].max_size, stats[i].reserved_size);
     }
     ei_free(stats);

     // Grab the heap statistics
     mbed_stats_heap_t heap_stats;
     mbed_stats_heap_get(&heap_stats);
     ei_printf("Heap size: %lu / %lu bytes (max: %lu)\r\n", heap_stats.current_size, heap_stats.reserved_size, heap_stats.max_size);
 }

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {
    if (is_initialised) return true;

    if (is_ll_initialised == false) {
        int r = cam.begin(CAMERA_R320x320, 30);
        if (r != 0) {
            ei_printf("ERR: Failed to initialise camera\r\n");
            return false;
        }
        is_ll_initialised = true;
    }

    // initialize frame buffer
#if defined(EI_CAMERA_FRAME_BUFFER_SDRAM) || defined(EI_CAMERA_FRAME_BUFFER_HEAP)
#ifdef EI_CAMERA_FRAME_BUFFER_SDRAM
    ei_camera_frame_mem = (uint8_t *) SDRAM.malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS + 32 /*alignment*/);
#else
    ei_camera_frame_mem = (uint8_t *) ei_malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS + 32 /*alignment*/);
#endif
    if(ei_camera_frame_mem == NULL) {
        ei_printf("failed to create ei_camera_frame_mem\r\n");
        return false;
    }
    ei_camera_frame_buffer = (uint8_t *)ALIGN_PTR((uintptr_t)ei_camera_frame_mem, 32);
#endif

    is_initialised = true;

    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

#if defined(EI_CAMERA_FRAME_BUFFER_SDRAM) || defined(EI_CAMERA_FRAME_BUFFER_HEAP)
#ifdef EI_CAMERA_FRAME_BUFFER_SDRAM
    SDRAM.free(ei_camera_frame_mem);
#else
    ei_free(ei_camera_frame_mem);
#endif
    ei_camera_frame_mem = NULL;
    ei_camera_frame_buffer = NULL;
#endif

    is_initialised = false;
}

/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;
    bool do_crop = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    EiDevice.set_state(eiStateSampling);

    int snapshot_response = cam.grab(ei_camera_frame_buffer);
    if (snapshot_response != 0) {
        ei_printf("ERR: Failed to get snapshot (%d)\r\n", snapshot_response);
        return false;
    }

    uint32_t resize_col_sz;
    uint32_t resize_row_sz;
    // choose resize dimensions
    int res = calculate_resize_dimensions(img_width, img_height, &resize_col_sz, &resize_row_sz, &do_resize);
    if (res) {
        ei_printf("ERR: Failed to calculate resize dimensions (%d)\r\n", res);
        return false;
    }

    if ((img_width != resize_col_sz)
        || (img_height != resize_row_sz)) {
        do_crop = true;
    }

    // The following variables should always be assigned
    // if this routine is to return true
    // cutout values
    ei_camera_capture_out = ei_camera_frame_buffer;

    if (do_resize) {
        //ei_printf("resize cols: %d, rows: %d\r\n", resize_col_sz,resize_row_sz);
        resizeImage(EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
                    ei_camera_frame_buffer,
                    resize_col_sz, resize_row_sz,
                    ei_camera_frame_buffer,
                    8);
    }

    if (do_crop) {
        uint32_t crop_col_sz;
        uint32_t crop_row_sz;
        uint32_t crop_col_start;
        uint32_t crop_row_start;
        crop_row_start = (resize_row_sz - img_height) / 2;
        crop_col_start = (resize_col_sz - img_width) / 2;
        crop_col_sz = img_width;
        crop_row_sz = img_height;

        if (out_buf) ei_camera_capture_out = out_buf;

        //ei_printf("crop cols: %d, rows: %d\r\n", crop_col_sz,crop_row_sz);
        cropImage(resize_col_sz, resize_row_sz,
                  ei_camera_frame_buffer,
                  crop_col_start, crop_row_start,
                  crop_col_sz, crop_row_sz,
                  ei_camera_capture_out,
                  8);
    }

    EiDevice.set_state(eiStateIdle);

    return true;
}

/**
 * @brief      Takes a snapshot, base64 encodes and outputs it to uart
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 *
 * @retval     true if snapshot was taken successfully
 *
 */
bool ei_camera_take_snapshot_encode_and_output(size_t width, size_t height, bool use_max_baudrate)
{
    bool result = true;

    if (!prepare_snapshot(width, height, use_max_baudrate)) result = false;

    if (result) {
        if (!take_snapshot(width, height, true)) {
            result = false;
        }
    }

    finish_snapshot();
    //print_memory_info2();

    return result;
}

/**
 * @brief      Starts a snapshot stream, base64 encodes and outputs it to uart
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 *
 * @retval     true if successful and/or terminated gracefully
 *
 */
bool ei_camera_start_snapshot_stream_encode_and_output(size_t width, size_t height, bool use_max_baudrate)
{
    bool result = true;

    ei_printf("Starting snapshot stream...\r\n");

    if (!prepare_snapshot(width, height, use_max_baudrate)) result = false;

    while (result) {
        if (!take_snapshot(width, height, true)) {
            result = false;
        }

        bool stopped_by_user = false;
        while (ei_get_serial_available() > 0) {
            if (ei_get_serial_byte() == 'b') {
                stopped_by_user = true;
                break;
            }
        }

        if (stopped_by_user) {
            ei_printf("Snapshot streaming stopped by user\r\n");
            EiDevice.set_state(eiStateIdle);
            break;
        }
    }

    finish_snapshot();

    return result;
}

/**
 * @brief      Helper function: Takes a snapshot, base64 encodes and prints it to uart
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  print_oks     whether to print OK indicator or not for CLI
 *
 * @retval     bool
 *
 * @note       Expects the camera and `ei_camera_frame_buffer` buffer to be
 * initialised
 */
static bool take_snapshot(size_t width, size_t height, bool print_oks)
{
    if (print_oks) {
        ei_printf("OK\r\n");
    }

    //ei_printf("take snapshot cols: %d, rows: %d\r\n", width, height);
    if (ei_camera_capture(width, height, NULL) == false) {
        ei_printf("ERR: Failed to capture image\r\n");
        return false;
    }

    ei::signal_t signal;
    signal.total_length = width * height;
    signal.get_data = &ei_camera_cutout_get_data;

    size_t signal_chunk_size = 1024;

    // loop through the signal
    float *signal_buf = (float*)ei_malloc(signal_chunk_size * sizeof(float));
    if (!signal_buf) {
        ei_printf("ERR: Failed to allocate signal buffer\r\n");
        return false;
    }

    uint8_t *per_pixel_buffer = (uint8_t*)ei_malloc(513); // 171 x 3 pixels
    if (!per_pixel_buffer) {
        ei_free(signal_buf);
        ei_printf("ERR: Failed to allocate per_pixel buffer\r\n");
        return false;
    }

    size_t per_pixel_buffer_ix = 0;

    for (size_t ix = 0; ix < signal.total_length; ix += signal_chunk_size) {
        size_t items_to_read = signal_chunk_size;
        if (items_to_read > signal.total_length - ix) {
            items_to_read = signal.total_length - ix;
        }

        int r = signal.get_data(ix, items_to_read, signal_buf);
        if (r != 0) {
            ei_printf("ERR: Failed to get data from signal (%d)\r\n", r);
            ei_free(signal_buf);
            ei_free(per_pixel_buffer);
            break;
        }

        for (size_t px = 0; px < items_to_read; px++) {
            uint32_t pixel = static_cast<uint32_t>(signal_buf[px]);

            // grab rgb
            uint8_t r = static_cast<float>(pixel >> 16 & 0xff);
            uint8_t g = static_cast<float>(pixel >> 8 & 0xff);
            uint8_t b = static_cast<float>(pixel & 0xff);

            // is monochrome anyway now, so just print 1 pixel at a time
            const bool print_rgb = false;

            if (print_rgb) {
                per_pixel_buffer[per_pixel_buffer_ix + 0] = r;
                per_pixel_buffer[per_pixel_buffer_ix + 1] = g;
                per_pixel_buffer[per_pixel_buffer_ix + 2] = b;
                per_pixel_buffer_ix += 3;
            }
            else {
                per_pixel_buffer[per_pixel_buffer_ix + 0] = r;
                per_pixel_buffer_ix++;
            }

            if (per_pixel_buffer_ix >= 513) {
                const size_t base64_output_size = 684;

                char *base64_buffer = (char*)ei_malloc(base64_output_size);
                if (!base64_buffer) {
                    ei_printf("ERR: Cannot allocate base64 buffer of size %lu, out of memory\r\n", base64_output_size);
                    ei_free(signal_buf);
                    ei_free(per_pixel_buffer);
                    return false;
                }

                int r = base64_encode((const char*)per_pixel_buffer, per_pixel_buffer_ix, base64_buffer, base64_output_size);
                if (r < 0) {
                    ei_printf("ERR: Failed to base64 encode (%d)\r\n", r);
                    ei_free(signal_buf);
                    ei_free(per_pixel_buffer);
                    return false;
                }

                ei_write_string(base64_buffer, r);
                per_pixel_buffer_ix = 0;
                ei_free(base64_buffer);
            }
            EiDevice.set_state(eiStateUploading);
        }
    }

    const size_t new_base64_buffer_output_size = floor(per_pixel_buffer_ix / 3 * 4) + 4;
    char *base64_buffer = (char*)ei_malloc(new_base64_buffer_output_size);
    if (!base64_buffer) {
        ei_printf("ERR: Cannot allocate base64 buffer of size %lu, out of memory\r\n", new_base64_buffer_output_size);
        ei_free(signal_buf);
        ei_free(per_pixel_buffer);
        return false;
    }

    int r = base64_encode((const char*)per_pixel_buffer, per_pixel_buffer_ix, base64_buffer, new_base64_buffer_output_size);
    if (r < 0) {
        ei_printf("ERR: Failed to base64 encode (%d)\r\n", r);
        ei_free(signal_buf);
        ei_free(per_pixel_buffer);
        return false;
    }

    ei_write_string(base64_buffer, r);
    ei_printf("\r\n");

    ei_free(base64_buffer);
    ei_free(signal_buf);
    ei_free(per_pixel_buffer);
    EiDevice.set_state(eiStateIdle);

    if (print_oks) {
        ei_printf("OK\r\n");
    }

    return true;
}

static bool verify_inputs(size_t width, size_t height)
{
    const ei_device_snapshot_resolutions_t *list;
    size_t list_size;
    const char *color_depth;

    int dl = EiDevice.get_snapshot_list((const ei_device_snapshot_resolutions_t **)&list, &list_size, &color_depth);
    if (dl) { /* apparently false is OK here?! */
        ei_printf("ERR: Device has no snapshot feature\r\n");
        return false;
    }

    bool found_res = false;
    for (size_t ix = 0; ix < list_size; ix++) {
        if (list[ix].width == width && list[ix].height == height) {
            found_res = true;
        }
    }

    if (!found_res) {
        ei_printf("ERR: Invalid resolution %lux%lu\r\n", width, height);
        return false;
    }

    return true;
}


static bool prepare_snapshot(size_t width, size_t height, bool use_max_baudrate)
{
    if (!verify_inputs(width, height)) { return false; }

    if (ei_camera_init() == false) {
        ei_printf("ERR: Failed to initialize image sensor\r\n");
        return false;
    }

    return true;
}

static void finish_snapshot() {
    /* noop */
}

int ei_camera_cutout_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t bytes_left = length;
    size_t out_ptr_ix = 0;

    // read byte for byte
    while (bytes_left != 0) {

        // grab the value and convert to r/g/b
        uint8_t pixel = ei_camera_capture_out[offset];

        uint8_t r, g, b;
        mono_to_rgb(pixel, &r, &g, &b);

        // then convert to out_ptr format
        float pixel_f = (r << 16) + (g << 8) + b;
        out_ptr[out_ptr_ix] = pixel_f;

        // and go to the next pixel
        out_ptr_ix++;
        offset++;
        bytes_left--;
    }

    // and done!
    return 0;
}

// This include file works in the Arduino environment
// to define the Cortex-M intrinsics
#ifdef __ARM_FEATURE_SIMD32
#include <device.h>
#endif
// This needs to be < 16 or it won't fit. Cortex-M4 only has SIMD for signed multiplies
#define FRAC_BITS 14
#define FRAC_VAL (1<<FRAC_BITS)
#define FRAC_MASK (FRAC_VAL - 1)
//
// Resize
//
// Assumes that the destination buffer is dword-aligned
// Can be used to resize the image smaller or larger
// If resizing much smaller than 1/3 size, then a more rubust algorithm should average all of the pixels
// This algorithm uses bilinear interpolation - averages a 2x2 region to generate each new pixel
//
// Optimized for 32-bit MCUs
// supports 8 and 16-bit pixels
void resizeImage(int srcWidth, int srcHeight, uint8_t *srcImage, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp)
{
  uint32_t src_x_accum, src_y_accum; // accumulators and fractions for scaling the image
  uint32_t x_frac, nx_frac, y_frac, ny_frac;
  int x, y, ty, tx;

  if (iBpp != 8 && iBpp != 16)
     return;
  src_y_accum = FRAC_VAL/2; // start at 1/2 pixel in to account for integer downsampling which might miss pixels
  const uint32_t src_x_frac = (srcWidth * FRAC_VAL) / dstWidth;
  const uint32_t src_y_frac = (srcHeight * FRAC_VAL) / dstHeight;
  const uint32_t r_mask = 0xf800f800;
  const uint32_t g_mask = 0x07e007e0;
  const uint32_t b_mask = 0x001f001f;
  uint8_t *s, *d;
  uint16_t *s16, *d16;
  uint32_t x_frac2, y_frac2; // for 16-bit SIMD
  for (y=0; y < dstHeight; y++) {
    ty = src_y_accum >> FRAC_BITS; // src y
    y_frac = src_y_accum & FRAC_MASK;
    src_y_accum += src_y_frac;
    ny_frac = FRAC_VAL - y_frac; // y fraction and 1.0 - y fraction
    y_frac2 = ny_frac | (y_frac << 16); // for M4/M4 SIMD
    s = &srcImage[ty * srcWidth];
    s16 = (uint16_t *)&srcImage[ty * srcWidth * 2];
    d = &dstImage[y * dstWidth];
    d16 = (uint16_t *)&dstImage[y * dstWidth * 2];
    src_x_accum = FRAC_VAL/2; // start at 1/2 pixel in to account for integer downsampling which might miss pixels
    if (iBpp == 8) {
      for (x=0; x < dstWidth; x++) {
        uint32_t tx, p00,p01,p10,p11;
        tx = src_x_accum >> FRAC_BITS;
        x_frac = src_x_accum & FRAC_MASK;
        nx_frac = FRAC_VAL - x_frac; // x fraction and 1.0 - x fraction
        x_frac2 = nx_frac | (x_frac << 16);
        src_x_accum += src_x_frac;
        p00 = s[tx]; p10 = s[tx+1];
        p01 = s[tx+srcWidth]; p11 = s[tx+srcWidth+1];
#ifdef __ARM_FEATURE_SIMD32
        p00 = __SMLAD(p00 | (p10<<16), x_frac2, FRAC_VAL/2) >> FRAC_BITS; // top line
        p01 = __SMLAD(p01 | (p11<<16), x_frac2, FRAC_VAL/2) >> FRAC_BITS; // bottom line
        p00 = __SMLAD(p00 | (p01<<16), y_frac2, FRAC_VAL/2) >> FRAC_BITS; // combine
#else // generic C code
        p00 = ((p00 * nx_frac) + (p10 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // top line
        p01 = ((p01 * nx_frac) + (p11 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // bottom line
        p00 = ((p00 * ny_frac) + (p01 * y_frac) + FRAC_VAL/2) >> FRAC_BITS; // combine top + bottom
#endif // Cortex-M4/M7
        *d++ = (uint8_t)p00; // store new pixel
      } // for x
    } // 8-bpp
    else
    { // RGB565
      for (x=0; x < dstWidth; x++) {
        uint32_t tx, p00,p01,p10,p11;
        uint32_t r00, r01, r10, r11, g00, g01, g10, g11, b00, b01, b10, b11;
        tx = src_x_accum >> FRAC_BITS;
        x_frac = src_x_accum & FRAC_MASK;
        nx_frac = FRAC_VAL - x_frac; // x fraction and 1.0 - x fraction
        x_frac2 = nx_frac | (x_frac << 16);
        src_x_accum += src_x_frac;
        p00 = __builtin_bswap16(s16[tx]); p10 = __builtin_bswap16(s16[tx+1]);
        p01 = __builtin_bswap16(s16[tx+srcWidth]); p11 = __builtin_bswap16(s16[tx+srcWidth+1]);
#ifdef __ARM_FEATURE_SIMD32
        {
        p00 |= (p10 << 16);
        p01 |= (p11 << 16);
        r00 = (p00 & r_mask) >> 1; g00 = p00 & g_mask; b00 = p00 & b_mask;
        r01 = (p01 & r_mask) >> 1; g01 = p01 & g_mask; b01 = p01 & b_mask;
        r00 = __SMLAD(r00, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // top line
        r01 = __SMLAD(r01, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // bottom line
        r00 = __SMLAD(r00 | (r01<<16), y_frac2, FRAC_VAL/2) >> FRAC_BITS; // combine
        g00 = __SMLAD(g00, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // top line
        g01 = __SMLAD(g01, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // bottom line
        g00 = __SMLAD(g00 | (g01<<16), y_frac2, FRAC_VAL/2) >> FRAC_BITS; // combine
        b00 = __SMLAD(b00, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // top line
        b01 = __SMLAD(b01, x_frac2, FRAC_VAL/2) >> FRAC_BITS; // bottom line
        b00 = __SMLAD(b00 | (b01<<16), y_frac2, FRAC_VAL/2) >> FRAC_BITS; // combine
        }
#else // generic C code
        {
        r00 = (p00 & r_mask) >> 1; g00 = p00 & g_mask; b00 = p00 & b_mask;
        r10 = (p10 & r_mask) >> 1; g10 = p10 & g_mask; b10 = p10 & b_mask;
        r01 = (p01 & r_mask) >> 1; g01 = p01 & g_mask; b01 = p01 & b_mask;
        r11 = (p11 & r_mask) >> 1; g11 = p11 & g_mask; b11 = p11 & b_mask;
        r00 = ((r00 * nx_frac) + (r10 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // top line
        r01 = ((r01 * nx_frac) + (r11 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // bottom line
        r00 = ((r00 * ny_frac) + (r01 * y_frac) + FRAC_VAL/2) >> FRAC_BITS; // combine top + bottom
        g00 = ((g00 * nx_frac) + (g10 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // top line
        g01 = ((g01 * nx_frac) + (g11 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // bottom line
        g00 = ((g00 * ny_frac) + (g01 * y_frac) + FRAC_VAL/2) >> FRAC_BITS; // combine top + bottom
        b00 = ((b00 * nx_frac) + (b10 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // top line
        b01 = ((b01 * nx_frac) + (b11 * x_frac) + FRAC_VAL/2) >> FRAC_BITS; // bottom line
        b00 = ((b00 * ny_frac) + (b01 * y_frac) + FRAC_VAL/2) >> FRAC_BITS; // combine top + bottom
        }
#endif // Cortex-M4/M7
        r00 = (r00 << 1) & r_mask;
        g00 = g00 & g_mask;
        b00 = b00 & b_mask;
        p00 = (r00 | g00 | b00); // re-combine color components
        *d16++ = (uint16_t)__builtin_bswap16(p00); // store new pixel
      } // for x
    } // 16-bpp
  } // for y
} /* resizeImage() */
//
// Crop
//
// Assumes that the destination buffer is dword-aligned
// optimized for 32-bit MCUs
// Supports 8 and 16-bit pixels
//
void cropImage(int srcWidth, int srcHeight, uint8_t *srcImage, int startX, int startY, int dstWidth, int dstHeight, uint8_t *dstImage, int iBpp)
{
    uint32_t *s32, *d32;
    int x, y;

    if (startX < 0 || startX >= srcWidth || startY < 0 || startY >= srcHeight || (startX + dstWidth) > srcWidth || (startY + dstHeight) > srcHeight)
       return; // invalid parameters
    if (iBpp != 8 && iBpp != 16)
       return;

    if (iBpp == 8) {
      uint8_t *s, *d;
      for (y=0; y<dstHeight; y++) {
        s = &srcImage[srcWidth * (y + startY) + startX];
        d = &dstImage[(dstWidth * y)];
        x = 0;
        if ((intptr_t)s & 3 || (intptr_t)d & 3) { // either src or dst pointer is not aligned
          for (; x<dstWidth; x++) {
            *d++ = *s++; // have to do it byte-by-byte
          }
        } else {
          // move 4 bytes at a time if aligned or alignment not enforced
          s32 = (uint32_t *)s;
          d32 = (uint32_t *)d;
          for (; x<dstWidth-3; x+= 4) {
            *d32++ = *s32++;
          }
          // any remaining stragglers?
          s = (uint8_t *)s32;
          d = (uint8_t *)d32;
          for (; x<dstWidth; x++) {
            *d++ = *s++;
          }
        }
      } // for y
    } // 8-bpp
    else
    {
      uint16_t *s, *d;
      for (y=0; y<dstHeight; y++) {
        s = (uint16_t *)&srcImage[2 * srcWidth * (y + startY) + startX * 2];
        d = (uint16_t *)&dstImage[(dstWidth * y * 2)];
        x = 0;
        if ((intptr_t)s & 2 || (intptr_t)d & 2) { // either src or dst pointer is not aligned
          for (; x<dstWidth; x++) {
            *d++ = *s++; // have to do it 16-bits at a time
          }
        } else {
          // move 4 bytes at a time if aligned or alignment no enforced
          s32 = (uint32_t *)s;
          d32 = (uint32_t *)d;
          for (; x<dstWidth-1; x+= 2) { // we can move 2 pixels at a time
            *d32++ = *s32++;
          }
          // any remaining stragglers?
          s = (uint16_t *)s32;
          d = (uint16_t *)d32;
          for (; x<dstWidth; x++) {
            *d++ = *s++;
          }
        }
      } // for y
    } // 16-bpp case
} /* cropImage() */
