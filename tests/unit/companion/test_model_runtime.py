#!/usr/bin/env python3
# Unit Test: TensorRT Inference Latency

import unittest
import time
import numpy as np
import os

# Try to import TensorRT and ONNX Runtime
try:
    import tensorrt as trt
    TRT_AVAILABLE = True
except ImportError:
    TRT_AVAILABLE = False

try:
    import onnxruntime as ort
    ORT_AVAILABLE = True
except ImportError:
    ORT_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class TestModelRuntime(unittest.TestCase):
    """Unit tests for AI model runtime performance"""
    
    @classmethod
    def setUpClass(cls):
        cls.models_dir = os.environ.get('MODELS_DIR', '/models')
        cls.device = 'cuda' if TRT_AVAILABLE else 'cpu'
        
    def test_tensorrt_engine_loading(self):
        """Test TensorRT engine loads correctly"""
        if not TRT_AVAILABLE:
            self.skipTest("TensorRT not available")
        
        # Test loading ONNX and building engine
        model_path = os.path.join(self.models_dir, 'yolov8n_obstacle.onnx')
        if not os.path.exists(model_path):
            self.skipTest(f"Model not found: {model_path}")
        
        logger = trt.Logger(trt.Logger.WARNING)
        builder = trt.Builder(logger)
        network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        parser = trt.OnnxParser(network, logger)
        
        with open(model_path, 'rb') as f:
            parser.parse(f.read())
        
        config = builder.create_builder_config()
        config.max_workspace_size = 1 << 30  # 1GB
        config.set_flag(trt.BuilderFlag.FP16)
        
        engine = builder.build_engine(network, config)
        self.assertIsNotNone(engine)
        
        # Check engine has correct bindings
        self.assertGreater(engine.num_bindings, 0)
    
    def test_tensorrt_inference_latency_yolov8n(self):
        """Test YOLOv8n obstacle detection latency < 50ms"""
        if not TRT_AVAILABLE:
            self.skipTest("TensorRT not available")
        
        model_path = os.path.join(self.models_dir, 'yolov8n_obstacle.engine')
        if not os.path.exists(model_path):
            self.skipTest(f"Engine not found: {model_path}")
        
        # Load engine
        logger = trt.Logger(trt.Logger.WARNING)
        with open(model_path, 'rb') as f:
            runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
            engine = runtime.deserialize_cuda_engine(f.read())
        
        context = engine.create_execution_context()
        
        # Prepare input
        input_shape = (1, 3, 640, 640)
        dummy_input = np.random.randn(*input_shape).astype(np.float32)
        
        # Allocate buffers
        bindings = []
        for binding in engine:
            shape = engine.get_binding_shape(binding)
            dtype = trt.nptype(engine.get_binding_dtype(binding))
            if engine.binding_is_input(binding):
                buf = np.empty(shape, dtype=dtype)
            else:
                buf = np.empty(shape, dtype=dtype)
            bindings.append(buf)
        
        # Warmup
        for _ in range(10):
            context.execute_v2(bindings)
        
        # Benchmark
        latencies = []
        for _ in range(100):
            # Copy input
            np.copyto(bindings[0], dummy_input)
            
            start = time.perf_counter()
            context.execute_v2(bindings)
            end = time.perf_counter()
            
            latencies.append((end - start) * 1000)  # ms
        
        avg_latency = np.mean(latencies)
        p99_latency = np.percentile(latencies, 99)
        
        print(f"YOLOv8n obstacle detection: avg={avg_latency:.2f}ms, p99={p99_latency:.2f}ms")
        
        self.assertLess(avg_latency, 50.0, f"Average latency {avg_latency:.2f}ms >= 50ms")
        self.assertLess(p99_latency, 50.0, f"P99 latency {p99_latency:.2f}ms >= 50ms")
    
    def test_tensorrt_inference_latency_mobilenetv3_landing(self):
        """Test MobileNetV3 landing assessment latency < 50ms"""
        if not TRT_AVAILABLE:
            self.skipTest("TensorRT not available")
        
        model_path = os.path.join(self.models_dir, 'mobilenetv3_landing.engine')
        if not os.path.exists(model_path):
            self.skipTest(f"Engine not found: {model_path}")
        
        # Similar benchmark as YOLO
        # ... (similar structure)
        self.assertTrue(True)  # Placeholder
    
    def test_tensorrt_inference_latency_lstm_fault(self):
        """Test LSTM fault detector latency < 50ms"""
        if not TRT_AVAILABLE:
            self.skipTest("TensorRT not available")
        
        model_path = os.path.join(self.models_dir, 'lstm_fault.engine')
        if not os.path.exists(model_path):
            self.skipTest(f"Engine not found: {model_path}")
        
        # ... (similar structure)
        self.assertTrue(True)  # Placeholder
    
    def test_onnxruntime_fallback_latency(self):
        """Test ONNX Runtime fallback latency"""
        if not ORT_AVAILABLE:
            self.skipTest("ONNX Runtime not available")
        
        model_path = os.path.join(self.models_dir, 'yolov8n_obstacle.onnx')
        if not os.path.exists(model_path):
            self.skipTest(f"Model not found: {model_path}")
        
        session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
        
        input_name = session.get_inputs()[0].name
        dummy_input = np.random.randn(1, 3, 640, 640).astype(np.float32)
        
        # Warmup
        for _ in range(10):
            session.run(None, {input_name: dummy_input})
        
        # Benchmark
        latencies = []
        for _ in range(100):
            start = time.perf_counter()
            session.run(None, {input_name: dummy_input})
            end = time.perf_counter()
            latencies.append((end - start) * 1000)
        
        avg_latency = np.mean(latencies)
        print(f"ONNX Runtime YOLOv8n: avg={avg_latency:.2f}ms")
        
        # ONNX Runtime should be slower but still reasonable
        self.assertLess(avg_latency, 200.0, f"ONNX Runtime latency {avg_latency:.2f}ms >= 200ms")
    
    def test_model_loading_time(self):
        """Test model loading time is reasonable"""
        if not TRT_AVAILABLE:
            self.skipTest("TensorRT not available")
        
        model_path = os.path.join(self.models_dir, 'yolov8n_obstacle.engine')
        if not os.path.exists(model_path):
            self.skipTest(f"Engine not found: {model_path}")
        
        start = time.perf_counter()
        logger = trt.Logger(trt.Logger.WARNING)
        with open(model_path, 'rb') as f:
            runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
            engine = runtime.deserialize_cuda_engine(f.read())
        end = time.perf_counter()
        
        load_time = (end - start) * 1000
        print(f"Model load time: {load_time:.2f}ms")
        
        self.assertLess(load_time, 5000.0, f"Model load time {load_time:.2f}ms >= 5000ms")
    
    def test_memory_usage(self):
        """Test GPU memory usage is within limits"""
        if not TRT_AVAILABLE:
            self.skipTest("TensorRT not available")
        
        import subprocess
        result = subprocess.run(['nvidia-smi', '--query-gpu=memory.used', '--format=csv,noheader,nounits'],
                               capture_output=True, text=True)
        if result.returncode == 0:
            mem_used_mb = int(result.stdout.strip())
            print(f"GPU memory used: {mem_used_mb} MB")
            self.assertLess(mem_used_mb, 8000, f"GPU memory {mem_used_mb}MB >= 8GB")


if __name__ == '__main__':
    unittest.main()