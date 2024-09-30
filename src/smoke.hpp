#pragma once

#include <iostream>
#include <NvInfer.h>
#include <opencv2/core.hpp>
#include "camera_3d_detector/trt_modulated_deform_conv.hpp"


class Logger : public nvinfer1::ILogger {
  public:
    explicit Logger(Severity severity = Severity::kWARNING)
        : reportable_severity(severity) {}

    void log(Severity severity, const char* msg) noexcept {
        if (severity > reportable_severity) {
            return;
        }
        switch (severity) {
            case Severity::kINTERNAL_ERROR:
                std::cerr << "INTERNAL_ERROR: ";
                break;
            case Severity::kERROR:
                std::cerr << "ERROR: ";
                break;
            case Severity::kWARNING:
                std::cerr << "WARNING: ";
                break;
            case Severity::kINFO:
                std::cerr << "INFO: ";
                break;
            default:
                std::cerr << "UNKNOWN: ";
                break;
        }
        std::cerr << msg << std::endl;
    }
    Severity reportable_severity;
};

struct BboxDim {
    float x;
    float y;
    float z;
};

struct Bbox {
    int class_id;
    float x;
    float y;
    float z;
    float length;
    float width;
    float height;
    float angle;
};

class SMOKE {
  public:
    SMOKE();
    SMOKE(const std::string& engine_path, const cv::Mat& intrinsic);
          
    ~SMOKE();

    void Detect(const cv::Mat& raw_img);
    void LoadOnnx(const std::string& onnx_path);
    void LoadEngine(const std::string& engine_path);
    void prepare(const cv::Mat& intrinsic);
    std::vector<Bbox> detector_result;
    cv::Mat img_resize;
  private:
    void PostProcess(cv::Mat& input_img);

    Logger g_logger_;
    cudaStream_t stream_;
    nvinfer1::IHostMemory* plan_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IExecutionContext* context_;
    nvinfer1::IRuntime* runtime_;
    
    void* buffers_[4];
    int buffer_size_[4];
    std::vector<float> image_data_;
    std::vector<float> bbox_preds_;
    std::vector<float> topk_scores_;
    std::vector<float> topk_indices_;
    cv::Mat intrinsic_;
    std::vector<float> base_depth_;
    std::vector<BboxDim> base_dims_;
};
