#include "wheel_perception/core/ai_engine.hpp"
#include <fstream>
#include <iostream>
#include <vector>
#include <cuda_runtime.h>
#include <NvInfer.h>

// -------------------------------------------------------------------------
// 声明外部定义的 CUDA 启动函数 (来自 preprocess.cu)
// -------------------------------------------------------------------------
extern "C" void launch_preprocess_kernel(
    void* src_ptr, int src_w, int src_h, 
    void* dst_ptr, int dst_w, int dst_h, 
    cudaStream_t stream
);

// Logger 实现
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        // 只打印警告及以上，屏蔽 Info
        if (severity <= Severity::kWARNING) 
            std::cout << "[TRT] " << msg << std::endl;
    }
} gLogger;

namespace wheel_perception {
namespace core {

class AiEngine::Impl {
public:
    Config config_;
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    
    // Binding 管理
    int input_idx_ = -1;
    int output_idx_ = -1;
    void* bindings_[2] = {nullptr, nullptr}; // combined5.engine: 单输入、单输出
    
    // 维度信息
    nvinfer1::Dims input_dims_;
    cudaStream_t stream_ = nullptr;

    Impl(const Config& config) : config_(config) {}

    ~Impl() {
        if (bindings_[0]) cudaFree(bindings_[0]);
        if (bindings_[1]) cudaFree(bindings_[1]);
        if (stream_) cudaStreamDestroy(stream_);
    }

    bool initialize() {
        // 1. 加载模型文件
        std::ifstream file(config_.engine_path, std::ios::binary);
        if (!file.good()) {
            std::cerr << "[AiEngine] Error: Engine file not found: " << config_.engine_path << std::endl;
            return false;
        }
        file.seekg(0, file.end);
        size_t size = file.tellg();
        file.seekg(0, file.beg);
        std::vector<char> model_stream(size);
        file.read(model_stream.data(), size);
        file.close();

        // 2. 反序列化
        runtime_.reset(nvinfer1::createInferRuntime(gLogger));
        engine_.reset(runtime_->deserializeCudaEngine(model_stream.data(), size));
        if (!engine_) return false;
        context_.reset(engine_->createExecutionContext());

        // 3. 保持原工程中已验证可用的静态 binding 分配路径。
        const int nbBindings = engine_->getNbBindings();
        if (nbBindings != 2) {
            std::cerr << "[AiEngine] Error: expected one input and one output, got "
                      << nbBindings << " bindings" << std::endl;
            return false;
        }
        for (int i = 0; i < nbBindings; ++i) {
            if (engine_->bindingIsInput(i)) {
                input_idx_ = i;
                input_dims_ = engine_->getBindingDimensions(i);
                // 保留原实现的分配规则：batch 固定按 1 计算。部分旧引擎
                // 会在元数据中把 batch 写成 -1，但此前可直接 enqueue。
                const size_t input_size = static_cast<size_t>(1) * 3 *
                    input_dims_.d[2] * input_dims_.d[3] * sizeof(float);
                if (cudaMalloc(&bindings_[i], input_size) != cudaSuccess) return false;
                std::cout << "[AiEngine] Input Found: " << engine_->getBindingName(i)
                          << " Dims: " << input_dims_.d[3] << "x" << input_dims_.d[2]
                          << std::endl;
            } else {
                output_idx_ = i;
                const nvinfer1::Dims out_dims = engine_->getBindingDimensions(i);
                size_t volume = 1;
                for (int d = 0; d < out_dims.nbDims; ++d) {
                    volume *= static_cast<size_t>(out_dims.d[d] < 0 ? 1 : out_dims.d[d]);
                }
                if (cudaMalloc(&bindings_[i], volume * sizeof(float)) != cudaSuccess) {
                    return false;
                }
                std::cout << "[AiEngine] Output Found: " << engine_->getBindingName(i)
                          << std::endl;
            }
        }
        
        if (input_idx_ < 0 || output_idx_ < 0) {
            std::cerr << "[AiEngine] Error: Could not find valid input/output bindings!" << std::endl;
            return false;
        }

        return cudaStreamCreate(&stream_) == cudaSuccess;
    }

    bool infer(void* zed_raw_ptr, int zed_w, int zed_h) {
        if (!context_ || !zed_raw_ptr || zed_w <= 0 || zed_h <= 0) return false;

        // 1. GPU 预处理 (Zero-Copy)
        // 调用外部定义的 CUDA Kernel
        launch_preprocess_kernel(
            zed_raw_ptr, zed_w, zed_h, 
            bindings_[input_idx_], input_dims_.d[3], input_dims_.d[2], // dst_w, dst_h
            stream_
        );

        // 2. 异步推理
        const bool enqueued = context_->enqueueV2(bindings_, stream_, nullptr);
        if (!enqueued) {
            std::cerr << "[AiEngine] Error: TensorRT enqueueV2 failed" << std::endl;
            return false;
        }
        
        // 注意：这里不同步，让后续的 Fusion Kernel 继续在 stream_ 上排队执行效率最高
        // 但为了简单起见，且 Fusion 是在另一个 CUDA Stream (默认流) 执行的，
        // 这里需要同步一下，确保 AI 算完了 Fusion 才能读。
        return cudaStreamSynchronize(stream_) == cudaSuccess;
    }

    void* getOutputTensor() {
        return bindings_[output_idx_];
    }
    
    // [实现新增接口]
    void getInputResolution(int& w, int& h) {
        // NCHW 格式：0:Batch, 1:Channel, 2:Height, 3:Width
        h = input_dims_.d[2];
        w = input_dims_.d[3];
    }

};

// --- 外壳函数转发 ---
AiEngine::AiEngine(const Config& config) : impl_(std::make_unique<Impl>(config)) {}
AiEngine::~AiEngine() = default;
bool AiEngine::initialize() { return impl_->initialize(); }
bool AiEngine::infer(void* ptr, int w, int h) { return impl_->infer(ptr, w, h); }
void* AiEngine::getOutputTensor(const std::string&) { return impl_->getOutputTensor(); }

// [转发新增接口]
void AiEngine::getInputResolution(int& w, int& h) { impl_->getInputResolution(w, h); }
} // core
} // wheel_perception
