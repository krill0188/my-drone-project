#!/usr/bin/env python3
# Companion Computer Hardware Abstraction Layer

import os
import subprocess
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Dict, Any, List
from pathlib import Path
import json


@dataclass
class HardwareInfo:
    """Hardware identification"""
    platform: str  # "jetson_orin_nx", "snapdragon_flight", "unknown"
    cpu_model: str
    gpu_model: str
    memory_gb: float
    storage_gb: float
    jetpack_version: Optional[str] = None
    sdk_version: Optional[str] = None


@dataclass
class SensorConfig:
    """Sensor configuration"""
    has_rtk_gps: bool = False
    has_stereo_camera: bool = False
    has_lidar_2d: bool = False
    has_lidar_3d: bool = False
    has_uwb: bool = False
    has_5g_modem: bool = False
    has_ai_compute: bool = False
    camera_devices: List[str] = field(default_factory=list)
    serial_ports: List[str] = field(default_factory=list)


class CompanionHal(ABC):
    """Abstract HAL for companion computer"""
    
    @abstractmethod
    def detect_hardware(self) -> HardwareInfo:
        pass
    
    @abstractmethod
    def detect_sensors(self) -> SensorConfig:
        pass
    
    @abstractmethod
    def get_gpu_memory_usage(self) -> Dict[str, float]:
        pass
    
    @abstractmethod
    def get_cpu_temperature(self) -> Optional[float]:
        pass
    
    @abstractmethod
    def get_gpu_temperature(self) -> Optional[float]:
        pass
    
    @abstractmethod
    def set_power_mode(self, mode: str) -> bool:  # "max_perf", "balanced", "power_save"
        pass
    
    @abstractmethod
    def enable_jetson_clocks(self) -> bool:
        pass


class JetsonOrinHal(CompanionHal):
    """NVIDIA Jetson Orin NX HAL implementation"""
    
    def detect_hardware(self) -> HardwareInfo:
        # Read Jetson model
        model_file = Path("/proc/device-tree/model")
        model = "unknown"
        if model_file.exists():
            model = model_file.read_text().strip('\x00')
        
        # Read JetPack version
        jetpack = None
        jetpack_file = Path("/etc/nv_tegra_release")
        if jetpack_file.exists():
            jetpack = jetpack_file.read_text().strip()
        
        # Get memory
        mem_info = Path("/proc/meminfo").read_text()
        mem_gb = 0
        for line in mem_info.split('\n'):
            if line.startswith('MemTotal:'):
                mem_kb = int(line.split()[1])
                mem_gb = mem_kb / 1024 / 1024
                break
        
        return HardwareInfo(
            platform="jetson_orin_nx",
            cpu_model="ARM Cortex-A78AE",
            gpu_model="NVIDIA Ampere (1024 CUDA cores, 32 Tensor cores)",
            memory_gb=mem_gb,
            storage_gb=0,  # Would query disk
            jetpack_version=jetpack
        )
    
    def detect_sensors(self) -> SensorConfig:
        config = SensorConfig()
        
        # Check for cameras
        for i in range(10):
            if Path(f"/dev/video{i}").exists():
                config.camera_devices.append(f"/dev/video{i}")
        
        # Check for serial ports
        for port in Path("/dev").glob("ttyTHS*"):
            config.serial_ports.append(str(port))
        for port in Path("/dev").glob("ttyUSB*"):
            config.serial_ports.append(str(port))
        for port in Path("/dev").glob("ttyACM*"):
            config.serial_ports.append(str(port))
        
        # Detect specific hardware
        # RTK GPS: check for u-blox on serial
        # UWB: check for DW3000 on SPI
        # LiDAR: check for I2C/serial devices
        
        # For now, return defaults based on typical Orin setup
        config.has_ai_compute = True
        config.has_rtk_gps = True
        config.has_stereo_camera = len(config.camera_devices) >= 2
        config.has_uwb = True
        
        return config
    
    def get_gpu_memory_usage(self) -> Dict[str, float]:
        try:
            result = subprocess.run(
                ["tegrastats", "--interval", "1", "--logfile", "/dev/stdout"],
                capture_output=True, text=True, timeout=2
            )
            # Parse tegrastats output for GPU memory
            # Output format: "RAM 4096/8192MB ... GR3D_FREQ 99% ..."
            return {"used_mb": 0, "total_mb": 0, "percent": 0}
        except Exception:
            return {"used_mb": 0, "total_mb": 0, "percent": 0}
    
    def get_cpu_temperature(self) -> Optional[float]:
        thermal_zones = Path("/sys/class/thermal")
        for zone in thermal_zones.glob("thermal_zone*"):
            type_file = zone / "type"
            temp_file = zone / "temp"
            if type_file.exists() and temp_file.exists():
                if "cpu" in type_file.read_text().lower():
                    temp_mc = int(temp_file.read_text().strip())
                    return temp_mc / 1000.0
        return None
    
    def get_gpu_temperature(self) -> Optional[float]:
        thermal_zones = Path("/sys/class/thermal")
        for zone in thermal_zones.glob("thermal_zone*"):
            type_file = zone / "type"
            temp_file = zone / "temp"
            if type_file.exists() and temp_file.exists():
                if "gpu" in type_file.read_text().lower():
                    temp_mc = int(temp_file.read_text().strip())
                    return temp_mc / 1000.0
        return None
    
    def set_power_mode(self, mode: str) -> bool:
        modes = {
            "max_perf": "0",
            "balanced": "1", 
            "power_save": "2"
        }
        if mode not in modes:
            return False
        try:
            subprocess.run(["sudo", "nvpmodel", "-m", modes[mode]], check=True)
            return True
        except subprocess.CalledProcessError:
            return False
    
    def enable_jetson_clocks(self) -> bool:
        try:
            subprocess.run(["sudo", "jetson_clocks"], check=True)
            return True
        except subprocess.CalledProcessError:
            return False


class SnapdragonFlightHal(CompanionHal):
    """Qualcomm Snapdragon Flight RB5 HAL implementation"""
    
    def detect_hardware(self) -> HardwareInfo:
        return HardwareInfo(
            platform="snapdragon_flight",
            cpu_model="Qualcomm QRB5165",
            gpu_model="Adreno 650",
            memory_gb=8.0,
            storage_gb=128.0,
            sdk_version="RB5 SDK"
        )
    
    def detect_sensors(self) -> SensorConfig:
        config = SensorConfig()
        # Similar detection logic for RB5
        config.has_ai_compute = True
        config.has_5g_modem = True
        return config
    
    def get_gpu_memory_usage(self) -> Dict[str, float]:
        return {"used_mb": 0, "total_mb": 0, "percent": 0}
    
    def get_cpu_temperature(self) -> Optional[float]:
        return None
    
    def get_gpu_temperature(self) -> Optional[float]:
        return None
    
    def set_power_mode(self, mode: str) -> bool:
        return True
    
    def enable_jetson_clocks(self) -> bool:
        return True  # No-op for Snapdragon


def create_hal() -> CompanionHal:
    """Factory function to create appropriate HAL"""
    # Check platform
    if Path("/proc/device-tree/model").exists():
        model = Path("/proc/device-tree/model").read_text().strip('\x00')
        if "Jetson" in model or "Orin" in model:
            return JetsonOrinHal()
    
    # Check for Snapdragon
    if Path("/sys/firmware/devicetree/base/model").exists():
        model = Path("/sys/firmware/devicetree/base/model").read_text().strip('\x00')
        if "RB5" in model or "Snapdragon" in model:
            return SnapdragonFlightHal()
    
    # Default to Jetson
    return JetsonOrinHal()


# Global HAL instance
_hal_instance: Optional[CompanionHal] = None
_hal_lock = threading.Lock()


def get_hal() -> CompanionHal:
    global _hal_instance
    with _hal_lock:
        if _hal_instance is None:
            _hal_instance = create_hal()
        return _hal_instance