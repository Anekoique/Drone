# `Perception` PLAN `01`

> Status: Draft
> Feature: `perception`
> Iteration: `01`
> Owner: Executor
> Depends on:
> - Previous Plan: `00_PLAN.md`
> - Review: `00_REVIEW.md`
> - Master Directive: `00_MASTER.md`

---

## Summary

Revised perception plan with full TensorRT code analysis, plugin strategy,
modern CUDA CMake, explicit cv_py triage, and sub-phased implementation.
Splits into two sub-phases: 3a (non-CUDA perception) and 3b (TensorRT
detector with plugin).

## Log

[**Feature Introduce**]

Full analysis of cv_cpp/trtdet (~2800 lines) reveals the TensorRT stack is
substantially larger than round 00 assumed. The engine requires a custom
`YoloLayerPlugin` shared library to deserialize. The detector is organized
as a self-contained `detector/` subdirectory within perception.

cv_py `detect.py` (~600 lines) contribution is now fully triaged into
kept/deferred/dropped behaviors.

[**Review Adjustments**]

- R-001 (HIGH): Fixed. Full plugin strategy added — `yolo_layer.hpp/cu`
  migrated as part of detector, `REGISTER_TENSORRT_PLUGIN` ensures
  auto-registration at load time, plugin shared lib built alongside detector.
- R-002 (HIGH): Fixed. G-7 removed from Phase 3. Detection2DArray
  construction deferred to Phase 6 (mission node). Phase 3 outputs
  `std::vector<Detection>` only. A conversion utility
  `to_detection2d_array()` is provided but not a ROS publisher.
- R-003 (HIGH): Fixed. Modern CMake CUDA strategy: `enable_language(CUDA)`,
  `find_package(CUDAToolkit)`, explicit `find_library(nvinfer)` for TensorRT.
  No deprecated `FindCUDA`.
- R-004 (MEDIUM): Fixed. Full cv_py triage table added.
- R-005 (MEDIUM): Noted. Minimal geometry tests added for camera model
  pixel↔world round-trip.

[**Master Compliance**]

- M-001: Full TensorRT file inventory with plugin/include analysis. Detector
  organized as `detector/` subdirectory with 8 headers + 4 source files.
- M-002: Detailed detector implementation section with engine loading,
  CUDA preprocess, inference, NMS, and plugin registration code.
- M-003: cv_py triage table — inference logic from cv_py, TensorRT runtime
  from cv_cpp. Mission logic (servo, state) deferred to Phase 6.
- M-004: Whole-project context: perception is library-only, consumed by
  Phase 6 mission node via direct function calls (single process on Jetson).
- M-005: All 3 HIGH + 2 MEDIUM review findings addressed explicitly.

### Changes from Previous Round

[**Added**]
- Plugin migration strategy (yolo_layer.hpp/cu)
- Full cv_cpp file inventory and kept/dropped matrix
- Full cv_py behavior triage table
- Modern CMake CUDA strategy (CUDAToolkit, enable_language)
- Sub-phase split: 3a (non-CUDA) + 3b (TensorRT)
- `to_detection2d_array()` utility function
- Minimal geometry test fixtures

[**Changed**]
- Detector architecture: flat → `detector/` subdirectory with plugin
- G-7 removed from this phase → Phase 6
- CMake: `find_package(CUDA)` → `find_package(CUDAToolkit)` + `enable_language(CUDA)`

[**Removed**]
- G-7 (Detection2DArray publisher) — deferred to Phase 6
- model.cpp, calibrator.cpp — engine building, not needed for inference
- global.hpp — replaced by proper class state

[**Unresolved**]
Nothing.

### Response Matrix

| Source | ID | Decision | Resolution |
|--------|----|----------|------------|
| Review | R-001 | Accepted | Plugin library migrated with auto-registration |
| Review | R-002 | Accepted | G-7 deferred, conversion utility provided |
| Review | R-003 | Accepted | Modern CUDAToolkit CMake |
| Review | R-004 | Accepted | cv_py triage table added |
| Review | R-005 | Accepted | Minimal geometry test fixtures |
| Review | TR-1 | Accepted | Adapter utility, not publisher |
| Review | TR-2 | Accepted | Sub-phased: 3a non-CUDA, 3b TensorRT |
| Master | M-001 | Applied | Full TensorRT inventory |
| Master | M-002 | Applied | Detailed detector implementation |
| Master | M-003 | Applied | cv_py triage |
| Master | M-004 | Applied | Whole-project context |
| Master | M-005 | Applied | All review findings addressed |

---

## Spec

[**Goals**]

- G-1: Migrate camera driver (direct V4L2/OpenCV).
- G-2: Migrate TensorRT detector with plugin library from cv_cpp/trtdet.
- G-3: Migrate Kalman filter detection tracking from Yolo.h.
- G-4: Migrate camera geometry model from CameraGimbal.h Camera class.
- G-5: Migrate clustering from clustering.h/cpp (break OffboardControl dep).
- G-6: All non-CUDA code compiles on Docker. Full stack compiles on Jetson.
- G-7: Provide `to_detection2d_array()` conversion utility (not a publisher).

Non-goals:
- NG-1: No ROS node/publisher in this phase.
- NG-2: No gimbal control (Phase 4).
- NG-3: No airdrop/servo/state logic (Phase 6).
- NG-4: No model building (model.cpp, calibrator.cpp) — engines are pre-built.
- NG-5: No cv_imshow visualization.

[**Architecture**]

```
include/drone/perception/
├── camera_driver.hpp
├── detection_types.hpp          TargetClass enum, Detection struct
├── detection_filter.hpp         2D Kalman filter + tracking
├── camera_model.hpp             Pixel↔world, distortion, ENU/NED/ESD
├── clustering.hpp               Target center finding
├── detection_adapter.hpp        to_detection2d_array() utility
│
└── detector/                    TensorRT inference subsystem
    ├── detector.hpp             Main class: load engine + detect(frame)
    ├── config.hpp               constexpr model params
    ├── types.hpp                RawDetection struct, YoloKernel
    ├── preprocess.hpp           CUDA preprocess API
    ├── postprocess.hpp          NMS + bbox decode
    ├── cuda_utils.hpp           CUDA_CHECK macro
    ├── logging.hpp              TensorRT ILogger
    └── plugin/
        └── yolo_layer.hpp       YoloLayerPlugin + Creator

src/perception/
├── camera_driver.cpp
├── detection_filter.cpp
├── camera_model.cpp
├── clustering.cpp
├── detection_adapter.cpp
│
└── detector/
    ├── detector.cpp             Engine deserialize, buffer mgmt, inference
    ├── preprocess.cu            Warp-affine CUDA kernel
    ├── postprocess.cpp          NMS, bbox decode
    └── plugin/
        └── yolo_layer.cu        CalDetection kernel + plugin impl
```

[**Invariants**]

- I-1: `detector/` is self-contained — nothing outside it touches CUDA.
- I-2: Camera model is pure geometry — no ROS, no CUDA.
- I-3: Clustering depends only on Eigen — no OffboardControl.
- I-4: Package builds without CUDA (detector excluded).
- I-5: Plugin auto-registers via `REGISTER_TENSORRT_PLUGIN`.
- I-6: No global variables anywhere in perception.

[**Data Structure**]

cv_cpp/trtdet file triage:

| cv_cpp File | Action | Target | Notes |
|---|---|---|---|
| `trtInfer.h/cpp` | Keep | `detector/detector.hpp/cpp` | RAII, no globals |
| `preprocess.h` + `.cu` | Keep | `detector/preprocess.hpp` + `.cu` | Unchanged kernel |
| `postprocess.h/cpp` | Keep | `detector/postprocess.hpp/cpp` | Remove draw fns |
| `config.h` | Keep | `detector/config.hpp` | Macros → constexpr |
| `types.h` | Keep | `detector/types.hpp` | Add namespace |
| `cuda_utils.h` | Keep | `detector/cuda_utils.hpp` | Unchanged |
| `logging.h` | Keep | `detector/logging.hpp` | NVIDIA boilerplate |
| `macros.h` | Keep | `detector/logging.hpp` (merged) | TRT version compat |
| `plugin/yololayer.h` | Keep | `detector/plugin/yolo_layer.hpp` | Add namespace |
| `plugin/yololayer.cu` | Keep | `detector/plugin/yolo_layer.cu` | Unchanged kernel |
| `model.h/cpp` | Drop | — | Engine building only |
| `calibrator.h/cpp` | Drop | — | INT8 calibration only |
| `utils.h` | Drop | — | File reading, not needed |
| `global.hpp` | Drop | — | Global state, replaced |
| `image_pub.cpp` | Extracted | `camera_driver.hpp` | Capture only |
| `image_sub.cpp` | Drop | — | ROS node, not needed |

cv_py `detect.py` behavior triage:

| cv_py Behavior | Action | Target | Notes |
|---|---|---|---|
| Dual-model inference (circle + H) | Keep | `detector.hpp` | Two engine instances |
| Class ID mapping (circle/stuffed/H) | Keep | `detection_types.hpp` | TargetClass enum |
| Camera undistortion (remap) | Keep | `camera_model.hpp` | Before inference |
| Detection2DArray construction | Keep | `detection_adapter.hpp` | Utility function |
| Confidence threshold | Keep | `detector/config.hpp` | Config parameter |
| cv2.imshow visualization | Drop | — | Headless on Jetson |
| State-aware detection filtering | Defer | Phase 6 mission | Not perception concern |
| Servo control / airdrop trigger | Defer | Phase 6 mission | Not perception concern |
| H-target retention (last detected) | Defer | Phase 6 mission | State management |
| Rangefinder altitude check | Defer | Phase 6 mission | Not perception concern |
| Video recording | Drop | — | Debug feature |
| visualization_targets subscriber | Drop | — | Debug feature |

[**API Surface**]

```cpp
namespace drone {

// --- Camera ---
class CameraDriver {
public:
  explicit CameraDriver(int device_id = 0, int width = 1280, int height = 720);
  std::optional<cv::Mat> capture();
};

// --- Detector (conditional CUDA) ---
#ifdef DRONE_HAS_CUDA
class Detector {
public:
  // Loads two engines: circle model + H model (dual-model from cv_py)
  Detector(const std::string& circle_engine, const std::string& h_engine);
  std::vector<Detection> detect(const cv::Mat& frame);
};
#endif

// --- Filter ---
class DetectionFilter {
public:
  void update(const std::vector<Detection>& detections, double dt);
  std::optional<Eigen::Vector2d> get_filtered(TargetClass target) const;
  std::optional<Eigen::Vector2d> get_velocity(TargetClass target) const;
};

// --- Camera Model ---
class CameraModel {
public:
  void load_intrinsics(const std::string& config_file);
  void set_pose(const Eigen::Vector3d& position, const Eigen::Vector3d& rotation);
  cv::Mat undistort(const cv::Mat& frame) const;
  std::optional<Eigen::Vector3d> pixel_to_world(const Eigen::Vector2d& pixel, double h) const;
  std::optional<Eigen::Vector2d> world_to_pixel(const Eigen::Vector3d& point) const;
  double calculate_pixel_radius(const Eigen::Vector3d& center, double radius) const;
  double calculate_real_diameter(double pixel_diameter, double distance) const;
};

// --- Clustering ---
struct Target {
  Eigen::Vector3d position;
  double diameter;
  int cluster_id;
};
std::vector<Target> find_cluster_centers(const std::vector<Eigen::Vector3d>& points);

// --- Adapter ---
vision_msgs::msg::Detection2DArray to_detection2d_array(
  const std::vector<Detection>& detections, const rclcpp::Time& stamp);

}  // namespace drone
```

[**Constraints**]

- C-1: CUDA `.cu` files require nvcc via `enable_language(CUDA)`.
- C-2: TensorRT libs from JetPack — discovered via `find_library(nvinfer)`.
- C-3: Build works WITHOUT CUDA on Docker/CI.
- C-4: Camera model preserves ENU/NED/ESD transforms.
- C-5: Clustering must not depend on OffboardControl.
- C-6: Plugin library must be linked before engine deserialization.

---

## Implement

### Execution Flow

Split into two sub-phases:

**Sub-phase 3a — Non-CUDA perception** (builds on Docker):
1. detection_types, detection_adapter
2. camera_driver
3. detection_filter (Kalman)
4. camera_model (extract from CameraGimbal.h)
5. clustering (break circular dep)
6. CMake + Docker build

**Sub-phase 3b — TensorRT detector** (builds on Jetson only):
7. detector/config, types, cuda_utils, logging
8. detector/preprocess.cu
9. detector/postprocess.cpp
10. detector/plugin/yolo_layer.hpp/cu
11. detector/detector.cpp (engine + dual-model)
12. CMake CUDA conditional
13. Jetson build verification

[**Failure Flow**]

1. No CUDA → detector excluded, rest compiles.
2. No TensorRT → same.
3. Plugin not found at runtime → engine deserialization fails with clear error.

[**State Transition**]

Not applicable.

### Implementation Plan

[**Step 1 — Detection types + adapter**]

`detection_types.hpp`: `Detection` struct, `TargetClass` enum.
`detection_adapter.hpp/cpp`: `to_detection2d_array()` conversion.

[**Step 2 — Camera driver**]

From `cv_cpp/image_pub.cpp`: extract V4L2 capture into RAII class.
No ROS publisher. Returns `std::optional<cv::Mat>`.

[**Step 3 — Detection filter**]

From `Yolo.h`: extract `KalmanFilter2D` class.
Per-class filter instances (circle, H).
Sort detections by distance to frame center.

[**Step 4 — Camera model**]

From `CameraGimbal.h` Camera class:
- `pixel_to_world()`: ray-plane intersection
- `world_to_pixel()`: perspective projection
- `undistort()`: OpenCV undistortPoints
- ENU/NED/ESD rotation matrices (const `Eigen::Matrix3d`)
- Load intrinsics from YAML via `ConfigLoader`
No gimbal (that's Phase 4).

[**Step 5 — Clustering**]

From `clustering.h/cpp`:
- Input: `std::vector<Eigen::Vector3d>` (not `OffboardControl*`)
- Remove global `Target_Samples`
- Remove `#include "OffboardControl.h"`

[**Step 6 — CMake (3a)**]

```cmake
find_package(OpenCV REQUIRED)

list(APPEND PERCEPTION_SOURCES
  src/perception/camera_driver.cpp
  src/perception/detection_filter.cpp
  src/perception/camera_model.cpp
  src/perception/clustering.cpp
  src/perception/detection_adapter.cpp
)
```

Add to `drone_utils` library. Docker build verification.

[**Step 7 — Detector config + types**]

`detector/config.hpp`: replace `#define` with `constexpr`.
`detector/types.hpp`: `RawDetection` struct (TensorRT output format).
`detector/cuda_utils.hpp`: `CUDA_CHECK` macro.
`detector/logging.hpp`: merge `logging.h` + `macros.h`.

[**Step 8 — CUDA preprocess**]

Port `preprocess.cu` unchanged. RAII for pinned/device buffers.
Replace global `img_buffer_host`/`img_buffer_device` with class members.

[**Step 9 — NMS postprocess**]

Port `postprocess.cpp`. Remove `draw_bbox`, `draw_mask_bbox`.
Keep `nms()`, `get_rect()`.

[**Step 10 — Plugin**]

Port `plugin/yololayer.h/cu` → `detector/plugin/yolo_layer.hpp/cu`.
Keep `REGISTER_TENSORRT_PLUGIN(YoloPluginCreator)`.
Plugin auto-registers when the shared library loads.

[**Step 11 — Detector class**]

Port `trtInfer.h/cpp` → `detector/detector.hpp/cpp`.
Key changes:
- RAII for all CUDA/TensorRT resources (unique_ptr + custom deleters)
- Two engine instances (circle + H, from cv_py dual-model pattern)
- Remove global `Logger` → member variable
- Remove `find_target()` with global char arrays → return `vector<Detection>`
- Constructor loads engine + registers plugin

```cpp
class Detector {
public:
  Detector(const std::string& circle_engine, const std::string& h_engine);
  ~Detector();
  std::vector<Detection> detect(const cv::Mat& frame);

private:
  struct Engine;  // pimpl for TensorRT types
  std::unique_ptr<Engine> circle_engine_;
  std::unique_ptr<Engine> h_engine_;
  std::vector<Detection> run_inference(Engine& engine, const cv::Mat& frame);
};
```

[**Step 12 — CMake CUDA conditional**]

```cmake
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  find_library(NVINFER_LIB nvinfer PATHS /usr/lib/aarch64-linux-gnu)
  find_library(NVPARSERS_LIB nvparsers PATHS /usr/lib/aarch64-linux-gnu)

  if(NVINFER_LIB)
    set(DRONE_HAS_CUDA TRUE)
    add_definitions(-DDRONE_HAS_CUDA)

    add_library(drone_perception_cuda
      src/perception/detector/detector.cpp
      src/perception/detector/preprocess.cu
      src/perception/detector/postprocess.cpp
      src/perception/detector/plugin/yolo_layer.cu
    )
    target_link_libraries(drone_perception_cuda
      CUDA::cudart
      ${NVINFER_LIB}
      drone_utils
    )
  endif()
endif()
```

## Trade-offs

- T-1: **Sub-phase 3a/3b split vs single phase**
  - Split: 3a compiles on Docker (CI friendly), 3b Jetson-only.
  - Single: simpler plan but CI can't verify anything.
  - Decision: Split. CI verifies non-CUDA perception code.

- T-2: **Dual-model (circle + H) vs single model**
  - Dual: matches cv_py production setup, separate confidence tuning.
  - Single: simpler but requires retraining a unified model.
  - Decision: Dual. Use existing trained engines.

- T-3: **Pimpl for Detector vs direct TensorRT headers in public API**
  - Pimpl: keeps TensorRT/CUDA headers out of public API, consumers
    don't need CUDA to include `detector.hpp`.
  - Direct: simpler, but forces CUDA headers on all includers.
  - Decision: Pimpl. Only `detector.cpp` sees TensorRT internals.

---

## Validation

[**Unit Tests**]

- V-UT-1: Camera model pixel→world→pixel round-trip with known intrinsics.
- V-UT-2: Detection filter produces stable output from noisy input sequence.
- V-UT-3: Clustering finds 3 centers from known point cloud.

[**Integration Tests**]

- V-IT-1: `colcon build` succeeds without CUDA (Docker).
- V-IT-2: `colcon test` passes.
- V-IT-3: All perception headers compile standalone.

[**Failure / Robustness Validation**]

- V-F-1: Build succeeds with `DRONE_HAS_CUDA=FALSE`.
- V-F-2: Detector constructor fails gracefully when engine file missing.

[**Edge Case Validation**]

- V-E-1: `grep -r "OffboardControl" include/drone/perception/` → zero.
- V-E-2: `grep -r "using namespace" include/drone/perception/` → zero.
- V-E-3: No global variables in perception code.
- V-E-4: No CUDA headers included outside `detector/`.

[**Acceptance Mapping**]

| Goal / Constraint | Validation |
|-------------------|------------|
| G-1 | CameraDriver compiles (V-IT-1) |
| G-2 | Detector compiles with CUDA (Jetson), excluded without |
| G-3 | V-UT-2 (filter stability) |
| G-4 | V-UT-1 (pixel↔world round-trip) |
| G-5 | V-E-1 (no OffboardControl), V-UT-3 |
| G-6 | V-IT-1, V-F-1 |
| G-7 | detection_adapter.hpp provides conversion |
| C-1 | CMake enable_language(CUDA) |
| C-2 | find_library(nvinfer) |
| C-3 | V-F-1 |
| C-4 | V-UT-1 (geometry round-trip) |
| C-5 | V-E-1 |
| C-6 | Plugin REGISTER_TENSORRT_PLUGIN |
