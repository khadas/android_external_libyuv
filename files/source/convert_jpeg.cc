/*
 *  Copyright 2011 The LibYuv Project Authors. All rights reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS. All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "libyuv/convert.h"
#include "libyuv/convert_argb.h"

#include <pthread.h>
#include <sched.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/prctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sysconf.h>
#include <sys/types.h>
#include <utils/threads.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

#ifdef HAVE_JPEG
#include "libyuv/mjpeg_decoder.h"
#endif

#ifdef __cplusplus
namespace libyuv {
extern "C" {
#endif

#ifdef HAVE_JPEG
struct I420Buffers {
  uint8* y;
  int y_stride;
  uint8* u;
  int u_stride;
  uint8* v;
  int v_stride;
  int w;
  int h;
};

static void JpegCopyI420(void* opaque,
                         const uint8* const* data,
                         const int* strides,
                         int rows) {
  I420Buffers* dest = (I420Buffers*)(opaque);
  I420Copy(data[0], strides[0],
           data[1], strides[1],
           data[2], strides[2],
           dest->y, dest->y_stride,
           dest->u, dest->u_stride,
           dest->v, dest->v_stride,
           dest->w, rows);
  dest->y += rows * dest->y_stride;
  dest->u += ((rows + 1) >> 1) * dest->u_stride;
  dest->v += ((rows + 1) >> 1) * dest->v_stride;
  dest->h -= rows;
}

static void JpegI422ToNV21_flag(void* opaque,
                           const uint8* const* data,
                           const int* strides,
                           int rows, int skip_flag) {
  NV21Buffers* dest = static_cast<NV21Buffers*>(opaque);

  if (skip_flag == 0) {
        I422ToNV21(data[0], strides[0],
                   data[1], strides[1],
                   data[2], strides[2],
                   dest->y, dest->y_stride,
                   dest->uv, dest->uv_stride,
                   dest->w, rows);
  }
      dest->y += rows * dest->y_stride;
      dest->uv += rows * dest->uv_stride;
      dest->h -= rows;
}



static void JpegI422ToNV21(void* opaque,
                           const uint8* const* data,
                           const int* strides,
                           int rows) {
  NV21Buffers* dest = static_cast<NV21Buffers*>(opaque);
  I422ToNV21(data[0], strides[0],
             data[1], strides[1],
             data[2], strides[2],
             dest->y, dest->y_stride,
             dest->uv, dest->uv_stride,
             dest->w, rows);
  dest->y += rows * dest->y_stride;
  dest->uv += rows * dest->uv_stride;
  dest->h -= rows;
}




static void JpegI422ToI420(void* opaque,
                           const uint8* const* data,
                           const int* strides,
                           int rows) {
  I420Buffers* dest = (I420Buffers*)(opaque);
  I422ToI420(data[0], strides[0],
             data[1], strides[1],
             data[2], strides[2],
             dest->y, dest->y_stride,
             dest->u, dest->u_stride,
             dest->v, dest->v_stride,
             dest->w, rows);
  dest->y += rows * dest->y_stride;
  dest->u += ((rows + 1) >> 1) * dest->u_stride;
  dest->v += ((rows + 1) >> 1) * dest->v_stride;
  dest->h -= rows;
}

static void JpegI444ToI420(void* opaque,
                           const uint8* const* data,
                           const int* strides,
                           int rows) {
  I420Buffers* dest = (I420Buffers*)(opaque);
  I444ToI420(data[0], strides[0],
             data[1], strides[1],
             data[2], strides[2],
             dest->y, dest->y_stride,
             dest->u, dest->u_stride,
             dest->v, dest->v_stride,
             dest->w, rows);
  dest->y += rows * dest->y_stride;
  dest->u += ((rows + 1) >> 1) * dest->u_stride;
  dest->v += ((rows + 1) >> 1) * dest->v_stride;
  dest->h -= rows;
}

static void JpegI411ToI420(void* opaque,
                           const uint8* const* data,
                           const int* strides,
                           int rows) {
  I420Buffers* dest = (I420Buffers*)(opaque);
  I411ToI420(data[0], strides[0],
             data[1], strides[1],
             data[2], strides[2],
             dest->y, dest->y_stride,
             dest->u, dest->u_stride,
             dest->v, dest->v_stride,
             dest->w, rows);
  dest->y += rows * dest->y_stride;
  dest->u += ((rows + 1) >> 1) * dest->u_stride;
  dest->v += ((rows + 1) >> 1) * dest->v_stride;
  dest->h -= rows;
}

static void JpegI400ToI420(void* opaque,
                           const uint8* const* data,
                           const int* strides,
                           int rows) {
  I420Buffers* dest = (I420Buffers*)(opaque);
  I400ToI420(data[0], strides[0],
             dest->y, dest->y_stride,
             dest->u, dest->u_stride,
             dest->v, dest->v_stride,
             dest->w, rows);
  dest->y += rows * dest->y_stride;
  dest->u += ((rows + 1) >> 1) * dest->u_stride;
  dest->v += ((rows + 1) >> 1) * dest->v_stride;
  dest->h -= rows;
}

// Query size of MJPG in pixels.
LIBYUV_API
int MJPGSize(const uint8* sample, size_t sample_size,
             int* width, int* height) {
  MJpegDecoder mjpeg_decoder;
  LIBYUV_BOOL ret = mjpeg_decoder.LoadFrame(sample, sample_size);
  if (ret) {
    *width = mjpeg_decoder.GetWidth();
    *height = mjpeg_decoder.GetHeight();
  }
  mjpeg_decoder.UnloadFrame();
  return ret ? 0 : -1;  // -1 for runtime failure.
}

// MJPG (Motion JPeg) to I420
// TODO(fbarchard): review w and h requirement. dw and dh may be enough.
LIBYUV_API
int MJPGToI420(const uint8* sample,
               size_t sample_size,
               uint8* y, int y_stride,
               uint8* u, int u_stride,
               uint8* v, int v_stride,
               int w, int h,
               int dw, int dh) {
  if (sample_size == kUnknownDataSize) {
    // ERROR: MJPEG frame size unknown
    return -1;
  }

  // TODO(fbarchard): Port MJpeg to C.
  MJpegDecoder mjpeg_decoder;
  LIBYUV_BOOL ret = mjpeg_decoder.LoadFrame(sample, sample_size);
  if (ret && (mjpeg_decoder.GetWidth() != w ||
              mjpeg_decoder.GetHeight() != h)) {
    // ERROR: MJPEG frame has unexpected dimensions
    mjpeg_decoder.UnloadFrame();
    return 1;  // runtime failure
  }
  if (ret) {
    I420Buffers bufs = { y, y_stride, u, u_stride, v, v_stride, dw, dh };
    // YUV420
    if (mjpeg_decoder.GetColorSpace() ==
            MJpegDecoder::kColorSpaceYCbCr &&
        mjpeg_decoder.GetNumComponents() == 3 &&
        mjpeg_decoder.GetVertSampFactor(0) == 2 &&
        mjpeg_decoder.GetHorizSampFactor(0) == 2 &&
        mjpeg_decoder.GetVertSampFactor(1) == 1 &&
        mjpeg_decoder.GetHorizSampFactor(1) == 1 &&
        mjpeg_decoder.GetVertSampFactor(2) == 1 &&
        mjpeg_decoder.GetHorizSampFactor(2) == 1) {
      ret = mjpeg_decoder.DecodeToCallback(&JpegCopyI420, &bufs, dw, dh);
    // YUV422
    } else if (mjpeg_decoder.GetColorSpace() ==
                   MJpegDecoder::kColorSpaceYCbCr &&
               mjpeg_decoder.GetNumComponents() == 3 &&
               mjpeg_decoder.GetVertSampFactor(0) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(0) == 2 &&
               mjpeg_decoder.GetVertSampFactor(1) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(1) == 1 &&
               mjpeg_decoder.GetVertSampFactor(2) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(2) == 1) {
      ret = mjpeg_decoder.DecodeToCallback(&JpegI422ToI420, &bufs, dw, dh);
    // YUV444
    } else if (mjpeg_decoder.GetColorSpace() ==
                   MJpegDecoder::kColorSpaceYCbCr &&
               mjpeg_decoder.GetNumComponents() == 3 &&
               mjpeg_decoder.GetVertSampFactor(0) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(0) == 1 &&
               mjpeg_decoder.GetVertSampFactor(1) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(1) == 1 &&
               mjpeg_decoder.GetVertSampFactor(2) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(2) == 1) {
      ret = mjpeg_decoder.DecodeToCallback(&JpegI444ToI420, &bufs, dw, dh);
    // YUV411
    } else if (mjpeg_decoder.GetColorSpace() ==
                   MJpegDecoder::kColorSpaceYCbCr &&
               mjpeg_decoder.GetNumComponents() == 3 &&
               mjpeg_decoder.GetVertSampFactor(0) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(0) == 4 &&
               mjpeg_decoder.GetVertSampFactor(1) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(1) == 1 &&
               mjpeg_decoder.GetVertSampFactor(2) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(2) == 1) {
      ret = mjpeg_decoder.DecodeToCallback(&JpegI411ToI420, &bufs, dw, dh);
    // YUV400
    } else if (mjpeg_decoder.GetColorSpace() ==
                   MJpegDecoder::kColorSpaceGrayscale &&
               mjpeg_decoder.GetNumComponents() == 1 &&
               mjpeg_decoder.GetVertSampFactor(0) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(0) == 1) {
      ret = mjpeg_decoder.DecodeToCallback(&JpegI400ToI420, &bufs, dw, dh);
    } else {
      // TODO(fbarchard): Implement conversion for any other colorspace/sample
      // factors that occur in practice. 411 is supported by libjpeg
      // ERROR: Unable to convert MJPEG frame because format is not supported
      mjpeg_decoder.UnloadFrame();
      return 1;
    }
  }
  return ret ? 0 : 1;
}


#define THREAD_NUM 3

LIBYUV_API
int MJPGToNV21(const uint8* sample,
               size_t sample_size,
               uint8* y, int y_stride,
               uint8* uv, int uv_stride,
               int w, int h,
               int dw, int dh) {
  if (sample_size == kUnknownDataSize) {
    // ERROR: MJPEG frame size unknown
    return -1;
  }

  // TODO(fbarchard): Port to C
  MJpegDecoder mjpeg_decoder;
  bool ret = mjpeg_decoder.LoadFrame(sample, sample_size);
  if (ret && (mjpeg_decoder.GetWidth() != w ||
              mjpeg_decoder.GetHeight() != h)) {
    // ERROR: MJPEG frame has unexpected dimensions
    mjpeg_decoder.UnloadFrame();
    return 1;  // runtime failure
  }
  if (ret) {
    NV21Buffers bufs = { y, y_stride, uv, uv_stride, dw, dh };
    // YUV422
    if (mjpeg_decoder.GetColorSpace() ==
                   MJpegDecoder::kColorSpaceYCbCr &&
               mjpeg_decoder.GetNumComponents() == 3 &&
               mjpeg_decoder.GetVertSampFactor(0) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(0) == 2 &&
               mjpeg_decoder.GetVertSampFactor(1) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(1) == 1 &&
               mjpeg_decoder.GetVertSampFactor(2) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(2) == 1) {
      ret = mjpeg_decoder.DecodeToCallback(&JpegI422ToNV21, &bufs, dw, dh);
    } else {
      // ERROR: Unable to convert MJPEG frame because format is not supported
      mjpeg_decoder.UnloadFrame();
      return 1;
    }
  }

  if (ret == true)
     return 0;
  else
     return -1;
}

static void set_thread_affinity(int cpu_num)
{
    int num = sysconf(_SC_NPROCESSORS_CONF);
    int i;
    cpu_set_t mask;
    cpu_set_t get;

    if(cpu_num>= num)
        cpu_num = 0;

    CPU_ZERO(&mask);
    CPU_SET(cpu_num, &mask);

    if (sched_setaffinity(0, sizeof(mask), &mask) == -1){
        ;//ALOGD("warning: could not set CPU affinity, continuing...");
    }
    CPU_ZERO(&get);
    if (sched_getaffinity(0, sizeof(get), &get) == -1){
        ;//ALOGD("warning: cound not get cpu affinity, continuing...");
    }
    for (i = 0; i < num; i++){
        if (CPU_ISSET(i, &get)){
            ;//ALOGD("this process %d is running processor : %d, prefer cpu is : %d",getpid(), i, cpu_num);
        }
    }
    return;
}

void * DecodeToCallbackMultiThD(void *arg)
{
    struct decode_multi_thd_s * pth_para = (struct decode_multi_thd_s * ) arg;
    set_thread_affinity(pth_para->id + 1);
    char thread_name[16]= "";
    pth_para->mjpeg_decoder.DecodeToCallbackMultiThD(pth_para, THREAD_NUM, &JpegI422ToNV21_flag, &(pth_para->bufs), pth_para->bufs.w, pth_para->bufs.h);
    return NULL;
}



LIBYUV_API
int MJPGToNV21_MultiThD(const uint8* sample,
               size_t sample_size,
               uint8* y, int y_stride,
               uint8* uv, int uv_stride,
               int w, int h,
               int dw, int dh) {

  pthread_attr_t attr;
  pthread_t tid[THREAD_NUM];
  decode_multi_thd_s decoder_th[THREAD_NUM];

  int policy, rs, priority;
  struct sched_param param;
  bool ret = -1;

  if (sample_size == kUnknownDataSize) {
    // ERROR: MJPEG frame size unknown
    return -1;
  }

  //TODO(fbarchard): Port to C

  NV21Buffers bufs = { y, y_stride, uv, uv_stride, dw, dh };
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);


  for (int i = THREAD_NUM - 1; i >= 0; i--) {
      decoder_th[i].id = i;

      ret = decoder_th[i].mjpeg_decoder.LoadFrame(sample, sample_size);
      if (ret && (decoder_th[i].mjpeg_decoder.GetWidth() != w ||
                  decoder_th[i].mjpeg_decoder.GetHeight() != h)) {
        // ERROR: MJPEG frame has unexpected dimensions
        decoder_th[i].mjpeg_decoder.UnloadFrame();
        return 1;  // runtime failure
      }

      if (ret) {
        decoder_th[i].bufs = bufs;
        // YUV422
        if (decoder_th[i].mjpeg_decoder.GetColorSpace() ==
                       MJpegDecoder::kColorSpaceYCbCr &&
                   decoder_th[i].mjpeg_decoder.GetNumComponents() == 3 &&
                   decoder_th[i].mjpeg_decoder.GetVertSampFactor(0) == 1 &&
                   decoder_th[i].mjpeg_decoder.GetHorizSampFactor(0) == 2 &&
                   decoder_th[i].mjpeg_decoder.GetVertSampFactor(1) == 1 &&
                   decoder_th[i].mjpeg_decoder.GetHorizSampFactor(1) == 1 &&
                   decoder_th[i].mjpeg_decoder.GetVertSampFactor(2) == 1 &&
                   decoder_th[i].mjpeg_decoder.GetHorizSampFactor(2) == 1) {
          if(pthread_create(&tid[i], &attr, DecodeToCallbackMultiThD, &decoder_th[i]) !=0){
              return -1;
          }
        } else {
          // ERROR: Unable to convert MJPEG frame because format is not supported

          decoder_th[i].mjpeg_decoder.UnloadFrame();
          return 1;
        }

      }
  }

  for (int i = 0; i < THREAD_NUM; i++) {
      pthread_join(tid[i], NULL);
  }
      pthread_attr_destroy(&attr);

  if (ret == true)
     return 0;
  else
     return -1;
}



#ifdef HAVE_JPEG
struct ARGBBuffers {
  uint8* argb;
  int argb_stride;
  int w;
  int h;
};

static void JpegI420ToARGB(void* opaque,
                         const uint8* const* data,
                         const int* strides,
                         int rows) {
  ARGBBuffers* dest = (ARGBBuffers*)(opaque);
  I420ToARGB(data[0], strides[0],
             data[1], strides[1],
             data[2], strides[2],
             dest->argb, dest->argb_stride,
             dest->w, rows);
  dest->argb += rows * dest->argb_stride;
  dest->h -= rows;
}

static void JpegI422ToARGB(void* opaque,
                           const uint8* const* data,
                           const int* strides,
                           int rows) {
  ARGBBuffers* dest = (ARGBBuffers*)(opaque);
  I422ToARGB(data[0], strides[0],
             data[1], strides[1],
             data[2], strides[2],
             dest->argb, dest->argb_stride,
             dest->w, rows);
  dest->argb += rows * dest->argb_stride;
  dest->h -= rows;
}

static void JpegI444ToARGB(void* opaque,
                           const uint8* const* data,
                           const int* strides,
                           int rows) {
  ARGBBuffers* dest = (ARGBBuffers*)(opaque);
  I444ToARGB(data[0], strides[0],
             data[1], strides[1],
             data[2], strides[2],
             dest->argb, dest->argb_stride,
             dest->w, rows);
  dest->argb += rows * dest->argb_stride;
  dest->h -= rows;
}

static void JpegI411ToARGB(void* opaque,
                           const uint8* const* data,
                           const int* strides,
                           int rows) {
  ARGBBuffers* dest = (ARGBBuffers*)(opaque);
  I411ToARGB(data[0], strides[0],
             data[1], strides[1],
             data[2], strides[2],
             dest->argb, dest->argb_stride,
             dest->w, rows);
  dest->argb += rows * dest->argb_stride;
  dest->h -= rows;
}

static void JpegI400ToARGB(void* opaque,
                           const uint8* const* data,
                           const int* strides,
                           int rows) {
  ARGBBuffers* dest = (ARGBBuffers*)(opaque);
  I400ToARGB(data[0], strides[0],
             dest->argb, dest->argb_stride,
             dest->w, rows);
  dest->argb += rows * dest->argb_stride;
  dest->h -= rows;
}

// MJPG (Motion JPeg) to ARGB
// TODO(fbarchard): review w and h requirement. dw and dh may be enough.
LIBYUV_API
int MJPGToARGB(const uint8* sample,
               size_t sample_size,
               uint8* argb, int argb_stride,
               int w, int h,
               int dw, int dh) {
  if (sample_size == kUnknownDataSize) {
    // ERROR: MJPEG frame size unknown
    return -1;
  }

  // TODO(fbarchard): Port MJpeg to C.
  MJpegDecoder mjpeg_decoder;
  LIBYUV_BOOL ret = mjpeg_decoder.LoadFrame(sample, sample_size);
  if (ret && (mjpeg_decoder.GetWidth() != w ||
              mjpeg_decoder.GetHeight() != h)) {
    // ERROR: MJPEG frame has unexpected dimensions
    mjpeg_decoder.UnloadFrame();
    return 1;  // runtime failure
  }
  if (ret) {
    ARGBBuffers bufs = { argb, argb_stride, dw, dh };
    // YUV420
    if (mjpeg_decoder.GetColorSpace() ==
            MJpegDecoder::kColorSpaceYCbCr &&
        mjpeg_decoder.GetNumComponents() == 3 &&
        mjpeg_decoder.GetVertSampFactor(0) == 2 &&
        mjpeg_decoder.GetHorizSampFactor(0) == 2 &&
        mjpeg_decoder.GetVertSampFactor(1) == 1 &&
        mjpeg_decoder.GetHorizSampFactor(1) == 1 &&
        mjpeg_decoder.GetVertSampFactor(2) == 1 &&
        mjpeg_decoder.GetHorizSampFactor(2) == 1) {
      ret = mjpeg_decoder.DecodeToCallback(&JpegI420ToARGB, &bufs, dw, dh);
    // YUV422
    } else if (mjpeg_decoder.GetColorSpace() ==
                   MJpegDecoder::kColorSpaceYCbCr &&
               mjpeg_decoder.GetNumComponents() == 3 &&
               mjpeg_decoder.GetVertSampFactor(0) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(0) == 2 &&
               mjpeg_decoder.GetVertSampFactor(1) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(1) == 1 &&
               mjpeg_decoder.GetVertSampFactor(2) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(2) == 1) {
      ret = mjpeg_decoder.DecodeToCallback(&JpegI422ToARGB, &bufs, dw, dh);
    // YUV444
    } else if (mjpeg_decoder.GetColorSpace() ==
                   MJpegDecoder::kColorSpaceYCbCr &&
               mjpeg_decoder.GetNumComponents() == 3 &&
               mjpeg_decoder.GetVertSampFactor(0) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(0) == 1 &&
               mjpeg_decoder.GetVertSampFactor(1) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(1) == 1 &&
               mjpeg_decoder.GetVertSampFactor(2) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(2) == 1) {
      ret = mjpeg_decoder.DecodeToCallback(&JpegI444ToARGB, &bufs, dw, dh);
    // YUV411
    } else if (mjpeg_decoder.GetColorSpace() ==
                   MJpegDecoder::kColorSpaceYCbCr &&
               mjpeg_decoder.GetNumComponents() == 3 &&
               mjpeg_decoder.GetVertSampFactor(0) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(0) == 4 &&
               mjpeg_decoder.GetVertSampFactor(1) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(1) == 1 &&
               mjpeg_decoder.GetVertSampFactor(2) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(2) == 1) {
      ret = mjpeg_decoder.DecodeToCallback(&JpegI411ToARGB, &bufs, dw, dh);
    // YUV400
    } else if (mjpeg_decoder.GetColorSpace() ==
                   MJpegDecoder::kColorSpaceGrayscale &&
               mjpeg_decoder.GetNumComponents() == 1 &&
               mjpeg_decoder.GetVertSampFactor(0) == 1 &&
               mjpeg_decoder.GetHorizSampFactor(0) == 1) {
      ret = mjpeg_decoder.DecodeToCallback(&JpegI400ToARGB, &bufs, dw, dh);
    } else {
      // TODO(fbarchard): Implement conversion for any other colorspace/sample
      // factors that occur in practice. 411 is supported by libjpeg
      // ERROR: Unable to convert MJPEG frame because format is not supported
      mjpeg_decoder.UnloadFrame();
      return 1;
    }
  }
  return ret ? 0 : 1;
}
#endif

#endif

#ifdef __cplusplus
}  // extern "C"
}  // namespace libyuv
#endif
