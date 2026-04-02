#include <algorithm>
#include <memory>
#include <new>
#include <vector>

#include "yolos.hpp"

namespace {
struct TrtYoloHandle {
  std::shared_ptr<tdt_radar::Infer<yolo::BoxArray>> detector;
};
}  // namespace

extern "C" void *fyt_trt_create(const char *engine_path, float conf, float nms) {
  if (engine_path == nullptr) {
    return nullptr;
  }

  auto *handle = new (std::nothrow) TrtYoloHandle();
  if (handle == nullptr) {
    return nullptr;
  }

  try {
    handle->detector = yolo::load(std::string(engine_path), yolo::Type::V8, conf, nms);
  } catch (...) {
    delete handle;
    return nullptr;
  }

  if (!handle->detector) {
    delete handle;
    return nullptr;
  }

  return static_cast<void *>(handle);
}

extern "C" void fyt_trt_destroy(void *ptr) {
  auto *handle = static_cast<TrtYoloHandle *>(ptr);
  delete handle;
}

extern "C" int fyt_trt_infer(
  void *ptr,
  const unsigned char *bgr,
  int width,
  int height,
  float *out_boxes,
  int max_det)
{
  if (ptr == nullptr || bgr == nullptr || out_boxes == nullptr || width <= 0 || height <= 0 || max_det <= 0) {
    return -1;
  }

  auto *handle = static_cast<TrtYoloHandle *>(ptr);
  if (!handle->detector) {
    return -2;
  }

  yolo::BoxArray boxes;
  try {
    boxes = handle->detector->forward(tdt_radar::Image{bgr, width, height});
  } catch (...) {
    return -3;
  }

  const int count = std::min(static_cast<int>(boxes.size()), max_det);
  for (int i = 0; i < count; ++i) {
    const auto &b = boxes[static_cast<size_t>(i)];
    float *dst = out_boxes + static_cast<size_t>(i) * 6;
    dst[0] = b.left;
    dst[1] = b.top;
    dst[2] = b.right;
    dst[3] = b.bottom;
    dst[4] = b.confidence;
    dst[5] = static_cast<float>(b.class_label);
  }
  return count;
}
