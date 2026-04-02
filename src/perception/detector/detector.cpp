#ifdef DRONE_HAS_CUDA

#include "drone/perception/detector/detector.hpp"

#include "drone/perception/detector/config.hpp"
#include "drone/perception/detector/cuda_utils.hpp"
#include "drone/perception/detector/postprocess.hpp"
#include "drone/perception/detector/preprocess.hpp"
#include "drone/perception/detector/types.hpp"

#include <NvInfer.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace drone
{

namespace
{

// TensorRT logger
class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TRT] " << msg << std::endl;
    }
  }
};

}  // namespace

/// Internal engine state for a single TensorRT model.
struct Detector::Impl
{
  struct Engine
  {
    nvinfer1::IRuntime * runtime = nullptr;
    nvinfer1::ICudaEngine * cuda_engine = nullptr;
    nvinfer1::IExecutionContext * context = nullptr;
    cudaStream_t stream{};
    float * gpu_buffers[2] = {nullptr, nullptr};
    float * cpu_output_buffer = nullptr;
    int class_id_offset = 0;  // offset for unified class IDs

    static constexpr int kOutputSize =
      detector::kMaxNumOutputBbox * sizeof(detector::RawDetection) / sizeof(float) + 1;

    void load(const std::string & engine_path, TrtLogger & logger)
    {
      std::ifstream file(engine_path, std::ios::binary);
      if (!file.good()) {
        throw std::runtime_error("Failed to open engine file: " + engine_path);
      }
      file.seekg(0, std::ios::end);
      size_t size = file.tellg();
      file.seekg(0, std::ios::beg);
      std::vector<char> data(size);
      file.read(data.data(), static_cast<std::streamsize>(size));

      runtime = nvinfer1::createInferRuntime(logger);
      assert(runtime);
      cuda_engine = runtime->deserializeCudaEngine(data.data(), size);
      assert(cuda_engine);
      context = cuda_engine->createExecutionContext();
      assert(context);

      CUDA_CHECK(cudaStreamCreate(&stream));
      detector::cuda_preprocess_init(detector::kMaxInputImageSize);

      // Allocate GPU buffers
      CUDA_CHECK(cudaMalloc(
        reinterpret_cast<void **>(&gpu_buffers[0]),
        detector::kBatchSize * 3 * detector::kInputH * detector::kInputW * sizeof(float)));
      CUDA_CHECK(cudaMalloc(
        reinterpret_cast<void **>(&gpu_buffers[1]),
        detector::kBatchSize * kOutputSize * sizeof(float)));
      cpu_output_buffer = new float[detector::kBatchSize * kOutputSize];
    }

    std::vector<Detection> infer(const cv::Mat & frame)
    {
      // CUDA preprocess
      detector::cuda_preprocess(
        frame.data, frame.cols, frame.rows, gpu_buffers[0], detector::kInputW, detector::kInputH,
        stream);
      CUDA_CHECK(cudaStreamSynchronize(stream));

      // Run TensorRT inference
      context->setInputTensorAddress(detector::kInputTensorName, gpu_buffers[0]);
      context->setOutputTensorAddress(detector::kOutputTensorName, gpu_buffers[1]);
      context->enqueueV3(stream);
      CUDA_CHECK(cudaMemcpyAsync(
        cpu_output_buffer, gpu_buffers[1], detector::kBatchSize * kOutputSize * sizeof(float),
        cudaMemcpyDeviceToHost, stream));
      cudaStreamSynchronize(stream);

      // NMS postprocess
      std::vector<detector::RawDetection> raw;
      detector::nms(raw, cpu_output_buffer, detector::kConfThresh, detector::kNmsThresh);

      // Convert to Detection format
      std::vector<Detection> result;
      result.reserve(raw.size());
      for (const auto & r : raw) {
        Detection d;
        d.cx = r.bbox[0];
        d.cy = r.bbox[1];
        d.w = r.bbox[2];
        d.h = r.bbox[3];
        d.confidence = r.conf;
        d.class_id = static_cast<int>(r.class_id) + class_id_offset;
        result.push_back(d);
      }
      return result;
    }

    ~Engine()
    {
      if (stream) cudaStreamDestroy(stream);
      if (gpu_buffers[0]) cudaFree(gpu_buffers[0]);
      if (gpu_buffers[1]) cudaFree(gpu_buffers[1]);
      delete[] cpu_output_buffer;
      detector::cuda_preprocess_destroy();
      delete context;
      delete cuda_engine;
      delete runtime;
    }
  };

  TrtLogger logger;
  std::unique_ptr<Engine> circle_engine;
  std::unique_ptr<Engine> h_engine;
};

Detector::Detector(const std::string & circle_engine_path, const std::string & h_engine_path)
: impl_(std::make_unique<Impl>())
{
  impl_->circle_engine = std::make_unique<Impl::Engine>();
  impl_->circle_engine->class_id_offset = 0;  // circle=0, stuffed=1
  impl_->circle_engine->load(circle_engine_path, impl_->logger);

  if (!h_engine_path.empty()) {
    impl_->h_engine = std::make_unique<Impl::Engine>();
    impl_->h_engine->class_id_offset = 2;  // H=2
    impl_->h_engine->load(h_engine_path, impl_->logger);
  }
}

Detector::~Detector() = default;
Detector::Detector(Detector &&) noexcept = default;
Detector & Detector::operator=(Detector &&) noexcept = default;

std::vector<Detection> Detector::detect(const cv::Mat & frame)
{
  auto result = impl_->circle_engine->infer(frame);
  if (impl_->h_engine) {
    auto h_result = impl_->h_engine->infer(frame);
    result.insert(result.end(), h_result.begin(), h_result.end());
  }
  return result;
}

}  // namespace drone

#endif  // DRONE_HAS_CUDA
