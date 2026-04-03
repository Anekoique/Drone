// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#ifdef DRONE_HAS_CUDA

#include <cuda_runtime.h>

#include <cstdint>

namespace drone::detector
{

/// Allocate GPU buffers for image preprocessing.
/// @param max_image_size Maximum number of pixels (width * height) to support.
void cuda_preprocess_init(int max_image_size);

/// Free GPU preprocessing buffers.
void cuda_preprocess_destroy();

/// Resize and normalize an image on GPU (letterbox + BGR->RGB + /255).
/// @param src Host pointer to BGR image data.
/// @param src_width Source image width.
/// @param src_height Source image height.
/// @param dst Device pointer to float output buffer (CHW format).
/// @param dst_width Network input width.
/// @param dst_height Network input height.
/// @param stream CUDA stream for async execution.
void cuda_preprocess(
  uint8_t * src, int src_width, int src_height, float * dst, int dst_width, int dst_height,
  cudaStream_t stream);

}  // namespace drone::detector

#endif  // DRONE_HAS_CUDA
