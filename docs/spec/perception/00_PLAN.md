# `Perception` PLAN `00`

> Status: Draft
> Feature: `perception`
> Iteration: `00`
> Owner: Executor
> Depends on:
> - Previous Plan: `none`
> - Review: `none`
> - Master Directive: `none`

---

## Summary

Migrate and unify the perception pipeline: camera capture, TensorRT YOLO
inference, detection filtering, camera geometry model, and target clustering.
This merges code from three sources: `cv_cpp/trtdet/` (TensorRT engine),
`cv_py/detect.py` (detection logic), and legacy drone code (`Yolo.h`,
`CameraGimbal.h`, `clustering.h/cpp`).

The result is a self-contained perception stack that runs entirely on-device
(Jetson), with no ROS topic overhead for the internal pipeline.

## Log

[**Feature Introduce**]

Phase 3 is the most complex migration. It unifies three separate codebases
into a coherent perception module:

1. **Camera driver**: direct V4L2/OpenCV capture (no ROS Image topic)
2. **TensorRT detector**: engine loading → CUDA preprocess → inference → NMS
3. **Detection filter**: 2D Kalman filter with tracking (from Yolo.h)
4. **Camera model**: pixel↔world transforms with distortion (from CameraGimbal.h)
5. **Clustering**: DBSCAN-like container center finding (from clustering.h)

Key design references:
- Autoware `autoware_tensorrt_yolox`: header layout under `include/<pkg>/`
- cv_cpp `trtdet/`: TensorRT inference pipeline (engine, preprocess.cu, postprocess, NMS)
- cv_py `detect.py`: dual-model inference, class mapping, state-aware publishing
- NVIDIA Isaac ROS: composable node patterns

[**Review Adjustments**]

N/A — first iteration.

[**Master Compliance**]

N/A — first iteration.

### Changes from Previous Round

[**Added**]
Everything — initial plan.

[**Changed**]
N/A

[**Removed**]
N/A

[**Unresolved**]
N/A

### Response Matrix

| Source | ID | Decision | Resolution |
|--------|----|----------|------------|
| — | — | — | First iteration, no prior findings |

---

## Spec

[**Goals**]

- G-1: Migrate camera capture as a direct V4L2/OpenCV class (no ROS topic).
- G-2: Port TensorRT inference from `cv_cpp/trtdet/` — engine loading, CUDA
  preprocessing, inference, NMS postprocessing.
- G-3: Migrate Kalman filter detection tracking from `Yolo.h`.
- G-4: Migrate camera geometry model from `CameraGimbal.h` Camera class
  (pixel↔world, distortion, ENU/NED transforms).
- G-5: Migrate clustering from `clustering.h/cpp` — break circular dep on
  OffboardControl.
- G-6: All code compiles, C++20, zero warnings. CUDA code compiles with nvcc.
- G-7: Detection output is `vision_msgs::msg::Detection2DArray` (standard ROS).

Non-goals:
- NG-1: No gimbal control (CameraGimbal ROS publisher) — that's Phase 4 drivers.
- NG-2: No airdrop decision logic — that's Phase 6 mission.
- NG-3: No servo control in detection pipeline — that's Phase 6.
- NG-4: No cv_imshow visualization — headless on Jetson.

[**Architecture**]

```
include/drone/perception/
├── camera_driver.hpp        V4L2/OpenCV capture
├── detector.hpp             TensorRT YOLO inference
├── detector_config.hpp      Model config (input size, thresholds, etc.)
├── detection_types.hpp      Detection struct, shared types
├── detection_filter.hpp     2D Kalman filter + tracking
├── camera_model.hpp         Pixel↔world transforms, distortion
└── clustering.hpp           Target center finding

src/perception/
├── camera_driver.cpp
├── detector.cpp             Engine loading, inference
├── preprocess.cu            CUDA warp-affine preprocessing
├── postprocess.cpp          NMS, bbox decoding
├── detection_filter.cpp
├── camera_model.cpp
└── clustering.cpp
```

Data flow (single process, no ROS topics internally):
```
CameraDriver::capture()
    → cv::Mat frame
    → Detector::detect(frame)
        → cuda_preprocess (GPU)
        → TensorRT inference (GPU)
        → NMS postprocess (CPU)
        → std::vector<Detection>
    → DetectionFilter::update(detections)
        → Kalman-filtered positions
    → CameraModel::pixel_to_world(position, height)
        → Eigen::Vector3d world_position
    → Clustering::find_centers(world_positions)
        → std::vector<Target> container_centers
```

[**Invariants**]

- I-1: No ROS topic overhead in the internal perception pipeline.
- I-2: Camera model is a pure geometry class — no ROS dependencies.
- I-3: Clustering depends only on Eigen — no OffboardControl dependency.
- I-4: TensorRT/CUDA code is conditionally compiled — package builds
  without CUDA (just without the detector).
- I-5: Detection types use standard `vision_msgs` for external interface.

[**Data Structure**]

```cpp
namespace drone {

struct Detection {
  float cx, cy;       // center x, y in pixels
  float w, h;         // width, height in pixels
  float confidence;
  int class_id;       // 0=circle, 1=stuffed, 2=H
};

enum class TargetClass : int {
  kCircle = 0,
  kStuffed = 1,
  kH = 2,
};

}  // namespace drone
```

Source mapping:

| Source | From | To | Notes |
|---|---|---|---|
| Camera | `cv_cpp/image_pub.cpp` | `camera_driver.hpp` | Direct capture, no ROS pub |
| TensorRT engine | `cv_cpp/trtdet/trtInfer.h/cpp` | `detector.hpp/cpp` | Clean class, no globals |
| CUDA preprocess | `cv_cpp/trtdet/preprocess.cu` | `preprocess.cu` | Warp-affine kernel |
| NMS postprocess | `cv_cpp/trtdet/postprocess.cpp` | `postprocess.cpp` | NMS + bbox decode |
| Config | `cv_cpp/trtdet/config.h` | `detector_config.hpp` | constexpr, no macros |
| Kalman filter | `Yolo.h` (KalmanFilter2D) | `detection_filter.hpp` | Extract from YOLO class |
| Detection subscriber | `Yolo.h` (YOLO class) | removed | No ROS subscriber needed |
| Camera model | `CameraGimbal.h` (Camera) | `camera_model.hpp` | Pure geometry, no ROS |
| Clustering | `clustering.h/cpp` | `clustering.hpp/cpp` | Remove OffboardControl dep |
| Class mapping | `cv_py/detect.py` | `detection_types.hpp` | circle/stuffed/H enum |

[**API Surface**]

```cpp
namespace drone {

class CameraDriver {
public:
  explicit CameraDriver(int device_id = 0, int width = 1280, int height = 720);
  std::optional<cv::Mat> capture();
};

class Detector {
public:
  explicit Detector(const std::string& engine_path);
  std::vector<Detection> detect(const cv::Mat& frame);
};

class DetectionFilter {
public:
  void update(const std::vector<Detection>& detections, double dt);
  std::optional<Eigen::Vector2d> get_filtered(TargetClass target) const;
};

class CameraModel {
public:
  void set_intrinsics(double fx, double fy, double cx, double cy, ...);
  void set_pose(const Eigen::Vector3d& position, const Eigen::Vector3d& rotation);
  std::optional<Eigen::Vector3d> pixel_to_world(const Eigen::Vector2d& pixel, double height) const;
  std::optional<Eigen::Vector2d> world_to_pixel(const Eigen::Vector3d& point) const;
};

std::vector<Target> find_cluster_centers(const std::vector<Eigen::Vector3d>& points);

}  // namespace drone
```

[**Constraints**]

- C-1: CUDA `.cu` files require nvcc — CMake must handle CUDA language.
- C-2: TensorRT headers and libs are provided by JetPack — not apt packages.
- C-3: Build must work WITHOUT CUDA/TensorRT on the dev workstation (Docker)
  for CI — use `find_package(CUDA QUIET)` conditional compilation.
- C-4: Camera model preserves the legacy ENU/NED/ESD coordinate transforms.
- C-5: Clustering must not depend on `OffboardControl.h`.

---

## Implement

### Execution Flow

[**Main Flow**]

1. Create directory structure.
2. Migrate detection types + config.
3. Migrate camera driver.
4. Migrate TensorRT detector (engine + preprocess.cu + postprocess + NMS).
5. Migrate detection filter (Kalman).
6. Migrate camera model (extract from CameraGimbal.h).
7. Migrate clustering (break circular dep).
8. Update CMakeLists.txt — conditional CUDA support.
9. Update package.xml.
10. Format + Docker build + test.

[**Failure Flow**]

1. CUDA not found → detector code excluded, rest compiles.
2. TensorRT not found → same.

[**State Transition**]

Not applicable.

### Implementation Plan

[**Step 1 — Types + Config**]

`detection_types.hpp`: `Detection` struct, `TargetClass` enum.
`detector_config.hpp`: constexpr input size, thresholds, tensor names.
Replace legacy `config.h` `#define` macros with constexpr.

[**Step 2 — Camera Driver**]

Simple OpenCV V4L2 capture class. No ROS. Configurable resolution.
From `cv_cpp/trtdet/image_pub.cpp` — extract capture logic only.

[**Step 3 — TensorRT Detector**]

Port from `cv_cpp/trtdet/`:
- `detector.cpp`: engine deserialization, buffer management, inference.
  Replace global `Logger` with member. RAII for CUDA/TensorRT resources.
- `preprocess.cu`: warp-affine kernel (unchanged, it's CUDA).
- `postprocess.cpp`: NMS + bbox decode. Remove draw functions.

Wrap in `#ifdef DRONE_HAS_CUDA` for conditional compilation.

[**Step 4 — Detection Filter**]

Extract `KalmanFilter2D` from `Yolo.h` into standalone header.
Extract tracking logic (sort by distance, per-class filtering).
Remove YOLO ROS node — detection comes from Detector directly.

[**Step 5 — Camera Model**]

Extract `Camera` class from `CameraGimbal.h`:
- `pixel_to_world()`, `world_to_pixel()`, distortion correction
- ENU/NED/ESD rotation matrices
- Remove gimbal ROS publisher (that's Phase 4)
- No `using namespace Eigen`

[**Step 6 — Clustering**]

Port `clustering.h/cpp`:
- Replace `#include "OffboardControl.h"` with direct Eigen types
- Input: `std::vector<Eigen::Vector3d>` points
- Output: `std::vector<Target>` cluster centers
- Remove global `Target_Samples` variable

[**Step 7 — CMakeLists.txt**]

```cmake
# Conditional CUDA/TensorRT support
find_package(CUDA QUIET)
find_package(OpenCV REQUIRED)

if(CUDA_FOUND)
  enable_language(CUDA)
  find_library(NVINFER_LIB nvinfer)
  if(NVINFER_LIB)
    set(DRONE_HAS_CUDA TRUE)
    add_definitions(-DDRONE_HAS_CUDA)
  endif()
endif()

# Perception sources (always compiled)
list(APPEND PERCEPTION_SOURCES
  src/perception/camera_driver.cpp
  src/perception/detection_filter.cpp
  src/perception/camera_model.cpp
  src/perception/clustering.cpp
  src/perception/postprocess.cpp
)

# CUDA sources (conditionally compiled)
if(DRONE_HAS_CUDA)
  list(APPEND PERCEPTION_SOURCES
    src/perception/detector.cpp
    src/perception/preprocess.cu
  )
endif()
```

[**Step 8 — Docker + CI**]

Docker image does NOT have CUDA — perception builds without detector.
Jetson builds with full CUDA/TensorRT support.

## Trade-offs

- T-1: **Direct capture vs ROS Image topic for camera**
  - Direct: zero latency, zero serialization. Single process on Jetson.
  - ROS topic: enables recording with `ros2 bag`, remote debugging.
  - Decision: Direct capture for production. Can optionally publish Image
    topic for debugging (controlled by config flag in a later phase).

- T-2: **Conditional CUDA compilation vs require CUDA always**
  - Conditional: CI works on Docker without CUDA, dev/test on any machine.
  - Required: simpler CMake, but blocks development without GPU.
  - Decision: Conditional. `#ifdef DRONE_HAS_CUDA` wraps detector code.

- T-3: **Port cv_cpp TensorRT code vs use existing ROS 2 TensorRT packages**
  - Port: full control, no external dependency, matches existing pipeline.
  - Existing (Isaac ROS, YOLOX-ROS): well-tested but adds large dependency
    tree and may not match our model format.
  - Decision: Port cv_cpp code. It already works with our trained models
    and is battle-tested in competition.

---

## Validation

[**Unit Tests**]

- V-UT-1: Deferred.

[**Integration Tests**]

- V-IT-1: `colcon build` succeeds without CUDA (Docker).
- V-IT-2: `colcon test` passes.
- V-IT-3: All perception headers compile standalone.

[**Failure / Robustness Validation**]

- V-F-1: Build succeeds with `DRONE_HAS_CUDA=FALSE`.
- V-F-2: Camera model pixel↔world round-trips correctly (code review).

[**Edge Case Validation**]

- V-E-1: `grep -r "OffboardControl" include/drone/perception/` → zero.
- V-E-2: `grep -r "using namespace" include/drone/perception/` → zero.
- V-E-3: No global variables in perception code.

[**Acceptance Mapping**]

| Goal / Constraint | Validation |
|-------------------|------------|
| G-1 | CameraDriver has no ROS deps (code review) |
| G-2 | V-IT-1 (compiles), CUDA conditional |
| G-3 | DetectionFilter standalone (V-IT-3) |
| G-4 | CameraModel standalone (V-IT-3, V-F-2) |
| G-5 | V-E-1 (no OffboardControl dep) |
| G-6 | V-IT-1 (zero warnings) |
| G-7 | detection_types.hpp uses standard types |
| C-1 | CMake CUDA handling |
| C-2 | Conditional find_library |
| C-3 | V-F-1 |
| C-4 | Camera model preserves transforms (code review) |
| C-5 | V-E-1 |
