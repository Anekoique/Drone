#ifdef DRONE_HAS_CUDA

#include "drone/perception/detector/cuda_utils.hpp"
#include "drone/perception/detector/preprocess.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstring>

namespace drone::detector
{

static uint8_t * img_buffer_host = nullptr;
static uint8_t * img_buffer_device = nullptr;

struct AffineMatrix
{
  float value[6];
};

__global__ void warpaffine_kernel(
  uint8_t * src, int src_line_size, int src_width, int src_height, float * dst, int dst_width,
  int dst_height, uint8_t const_value_st, AffineMatrix d2s, int edge)
{
  int position = blockDim.x * blockIdx.x + threadIdx.x;
  if (position >= edge) return;

  float m_x1 = d2s.value[0], m_y1 = d2s.value[1], m_z1 = d2s.value[2];
  float m_x2 = d2s.value[3], m_y2 = d2s.value[4], m_z2 = d2s.value[5];

  int dx = position % dst_width;
  int dy = position / dst_width;
  float src_x = m_x1 * dx + m_y1 * dy + m_z1 + 0.5f;
  float src_y = m_x2 * dx + m_y2 * dy + m_z2 + 0.5f;
  float c0, c1, c2;

  if (src_x <= -1 || src_x >= src_width || src_y <= -1 || src_y >= src_height) {
    c0 = c1 = c2 = const_value_st;
  } else {
    int y_low = floorf(src_y), x_low = floorf(src_x);
    int y_high = y_low + 1, x_high = x_low + 1;
    uint8_t cv_st[] = {const_value_st, const_value_st, const_value_st};
    float ly = src_y - y_low, lx = src_x - x_low;
    float hy = 1 - ly, hx = 1 - lx;
    float w1 = hy * hx, w2 = hy * lx, w3 = ly * hx, w4 = ly * lx;

    uint8_t *v1 = cv_st, *v2 = cv_st, *v3 = cv_st, *v4 = cv_st;
    if (y_low >= 0) {
      if (x_low >= 0) v1 = src + y_low * src_line_size + x_low * 3;
      if (x_high < src_width) v2 = src + y_low * src_line_size + x_high * 3;
    }
    if (y_high < src_height) {
      if (x_low >= 0) v3 = src + y_high * src_line_size + x_low * 3;
      if (x_high < src_width) v4 = src + y_high * src_line_size + x_high * 3;
    }
    c0 = w1 * v1[0] + w2 * v2[0] + w3 * v3[0] + w4 * v4[0];
    c1 = w1 * v1[1] + w2 * v2[1] + w3 * v3[1] + w4 * v4[1];
    c2 = w1 * v1[2] + w2 * v2[2] + w3 * v3[2] + w4 * v4[2];
  }

  // BGR to RGB + normalize
  float t = c2;
  c2 = c0;
  c0 = t;
  c0 /= 255.0f;
  c1 /= 255.0f;
  c2 /= 255.0f;

  // Planar layout: RRRGGGBBB
  int area = dst_width * dst_height;
  float * p0 = dst + dy * dst_width + dx;
  p0[0] = c0;
  p0[area] = c1;
  p0[2 * area] = c2;
}

void cuda_preprocess(
  uint8_t * src, int src_width, int src_height, float * dst, int dst_width, int dst_height,
  cudaStream_t stream)
{
  int img_size = src_width * src_height * 3;
  std::memcpy(img_buffer_host, src, img_size);
  CUDA_CHECK(
    cudaMemcpyAsync(img_buffer_device, img_buffer_host, img_size, cudaMemcpyHostToDevice, stream));

  AffineMatrix s2d{}, d2s{};
  float scale = std::min(
    static_cast<float>(dst_height) / src_height, static_cast<float>(dst_width) / src_width);
  s2d.value[0] = scale;
  s2d.value[1] = 0;
  s2d.value[2] = -scale * src_width * 0.5f + dst_width * 0.5f;
  s2d.value[3] = 0;
  s2d.value[4] = scale;
  s2d.value[5] = -scale * src_height * 0.5f + dst_height * 0.5f;

  cv::Mat m2x3_s2d(2, 3, CV_32F, s2d.value);
  cv::Mat m2x3_d2s(2, 3, CV_32F, d2s.value);
  cv::invertAffineTransform(m2x3_s2d, m2x3_d2s);
  std::memcpy(d2s.value, m2x3_d2s.ptr<float>(0), sizeof(d2s.value));

  int jobs = dst_height * dst_width;
  int threads = 256;
  int blocks = (jobs + threads - 1) / threads;
  warpaffine_kernel<<<blocks, threads, 0, stream>>>(
    img_buffer_device, src_width * 3, src_width, src_height, dst, dst_width, dst_height, 128, d2s,
    jobs);
}

void cuda_preprocess_init(int max_image_size)
{
  CUDA_CHECK(cudaMallocHost(reinterpret_cast<void **>(&img_buffer_host), max_image_size * 3));
  CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&img_buffer_device), max_image_size * 3));
}

void cuda_preprocess_destroy()
{
  CUDA_CHECK(cudaFree(img_buffer_device));
  CUDA_CHECK(cudaFreeHost(img_buffer_host));
}

}  // namespace drone::detector

#endif  // DRONE_HAS_CUDA
