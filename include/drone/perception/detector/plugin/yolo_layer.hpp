// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#ifdef DRONE_HAS_CUDA

#include "drone/perception/detector/types.hpp"

#include <NvInfer.h>

#include <string>
#include <vector>

// TensorRT version compatibility macros
#if NV_TENSORRT_MAJOR >= 8
#define TRT_NOEXCEPT noexcept
#define TRT_CONST_ENQUEUE const
#else
#define TRT_NOEXCEPT
#define TRT_CONST_ENQUEUE
#endif

#ifndef API
#define API
#endif

namespace nvinfer1
{

/// Custom TensorRT plugin implementing the YOLO detection output layer.
/// Decodes anchor-based predictions into bounding boxes on GPU.
class API YoloLayerPlugin : public IPluginV2IOExt
{
public:
  /// Construct from parameters.
  /// @param classCount Number of object classes.
  /// @param netWidth Network input width.
  /// @param netHeight Network input height.
  /// @param maxOut Maximum output detections.
  /// @param is_segmentation True if the model includes segmentation masks.
  /// @param vYoloKernel Per-layer anchor definitions.
  YoloLayerPlugin(
    int classCount, int netWidth, int netHeight, int maxOut, bool is_segmentation,
    const std::vector<drone::detector::YoloKernel> & vYoloKernel);

  /// Deserialize from a serialized plugin buffer.
  YoloLayerPlugin(const void * data, size_t length);
  ~YoloLayerPlugin();

  int getNbOutputs() const TRT_NOEXCEPT override { return 1; }
  Dims getOutputDimensions(int index, const Dims * inputs, int nbInputDims) TRT_NOEXCEPT override;
  int initialize() TRT_NOEXCEPT override;
  void terminate() TRT_NOEXCEPT override {}
  size_t getWorkspaceSize(int /*maxBatchSize*/) const TRT_NOEXCEPT override { return 0; }

  /// Execute the YOLO decode kernel on GPU.
  int enqueue(
    int batchSize, const void * const * inputs, void * TRT_CONST_ENQUEUE * outputs,
    void * workspace, cudaStream_t stream) TRT_NOEXCEPT override;

  size_t getSerializationSize() const TRT_NOEXCEPT override;
  void serialize(void * buffer) const TRT_NOEXCEPT override;

  bool supportsFormatCombination(
    int pos, const PluginTensorDesc * inOut, int /*nbInputs*/,
    int /*nbOutputs*/) const TRT_NOEXCEPT override
  {
    return inOut[pos].format == TensorFormat::kLINEAR && inOut[pos].type == DataType::kFLOAT;
  }

  const char * getPluginType() const TRT_NOEXCEPT override;
  const char * getPluginVersion() const TRT_NOEXCEPT override;
  void destroy() TRT_NOEXCEPT override;
  IPluginV2IOExt * clone() const TRT_NOEXCEPT override;
  void setPluginNamespace(const char * ns) TRT_NOEXCEPT override;
  const char * getPluginNamespace() const TRT_NOEXCEPT override;

  DataType getOutputDataType(int index, const DataType * inputTypes, int nbInputs) const
    TRT_NOEXCEPT override;
  bool isOutputBroadcastAcrossBatch(
    int outputIndex, const bool * inputIsBroadcasted, int nbInputs) const TRT_NOEXCEPT override;
  bool canBroadcastInputAcrossBatch(int inputIndex) const TRT_NOEXCEPT override;
  void attachToContext(cudnnContext * cudnn, cublasContext * cublas, IGpuAllocator * allocator)
    TRT_NOEXCEPT override;
  void configurePlugin(
    const PluginTensorDesc * in, int nbInput, const PluginTensorDesc * out,
    int nbOutput) TRT_NOEXCEPT override;
  void detachFromContext() TRT_NOEXCEPT override;

private:
  void forwardGpu(const float * const * inputs, float * output, cudaStream_t stream, int batchSize);

  int mThreadCount = 256;
  const char * mPluginNamespace = "";
  int mKernelCount = 0;
  int mClassCount = 0;
  int mYoloV5NetWidth = 0;
  int mYoloV5NetHeight = 0;
  int mMaxOutObject = 0;
  bool is_segmentation_ = false;
  std::vector<drone::detector::YoloKernel> mYoloKernel;
  void ** mAnchor = nullptr;
};

/// Factory for creating and deserializing YoloLayerPlugin instances.
class API YoloPluginCreator : public IPluginCreator
{
public:
  YoloPluginCreator();
  ~YoloPluginCreator() override = default;

  const char * getPluginName() const TRT_NOEXCEPT override;
  const char * getPluginVersion() const TRT_NOEXCEPT override;
  const PluginFieldCollection * getFieldNames() TRT_NOEXCEPT override;

  /// Create a new plugin instance from field collection.
  IPluginV2IOExt * createPlugin(const char * name, const PluginFieldCollection * fc)
    TRT_NOEXCEPT override;

  /// Deserialize a plugin from serialized data.
  IPluginV2IOExt * deserializePlugin(
    const char * name, const void * serialData, size_t serialLength) TRT_NOEXCEPT override;

  void setPluginNamespace(const char * ns) TRT_NOEXCEPT override { mNamespace = ns; }
  const char * getPluginNamespace() const TRT_NOEXCEPT override { return mNamespace.c_str(); }

private:
  std::string mNamespace;
  static PluginFieldCollection mFC;
  static std::vector<PluginField> mPluginAttributes;
};

REGISTER_TENSORRT_PLUGIN(YoloPluginCreator);

}  // namespace nvinfer1

#endif  // DRONE_HAS_CUDA
