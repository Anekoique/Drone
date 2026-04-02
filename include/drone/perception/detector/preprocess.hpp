#pragma once

#ifdef DRONE_HAS_CUDA

#include <cuda_runtime.h>

#include <cstdint>

namespace drone::detector
{

void cuda_preprocess_init(int max_image_size);
void cuda_preprocess_destroy();
void cuda_preprocess(
  uint8_t * src, int src_width, int src_height, float * dst, int dst_width, int dst_height,
  cudaStream_t stream);

}  // namespace drone::detector

#endif  // DRONE_HAS_CUDA
