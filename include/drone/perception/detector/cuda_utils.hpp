// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#ifdef DRONE_HAS_CUDA

#include <cuda_runtime_api.h>

#include <cassert>
#include <iostream>

/// Check a CUDA call and abort with diagnostics on failure.
// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#define CUDA_CHECK(call)                                                                        \
  do {                                                                                          \
    cudaError_t error_code = (call);                                                            \
    if (error_code != cudaSuccess) {                                                            \
      std::cerr << "CUDA error " << error_code << " at " << __FILE__ << ":" << __LINE__ << ": " \
                << cudaGetErrorString(error_code) << std::endl;                                 \
      assert(false);                                                                            \
    }                                                                                           \
  } while (0)
// NOLINTEND(cppcoreguidelines-macro-usage)

#endif  // DRONE_HAS_CUDA
