# Configuration management for companion computer (Python)

import yaml
from pathlib import Path
from typing import Any, Dict, List, Optional, Union, TypeVar, Generic
from dataclasses import dataclass, field, asdict
import threading
from copy import deepcopy

T = TypeVar('T')


@dataclass
class ConfigSchema:
    """Schema definition for a configuration parameter"""
    name: str
    description: str
    default_value: Any
    min_value: Optional[Any] = None
    max_value: Optional[Any] = None
    required: bool = False
    reboot_required: bool = False
    value_type: Optional[type] = None


class ConfigManager:
    """Thread-safe configuration manager with YAML persistence"""
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        self._config: Dict[str, Any] = {}
        self._schemas: Dict[str, ConfigSchema] = {}
        self._filepath: Optional[Path] = None
        self._initialized = True
    
    def load_from_file(self, filepath: Union[str, Path]) -> bool:
        """Load configuration from YAML file"""
        self._filepath = Path(filepath)
        try:
            if self._filepath.exists():
                with open(self._filepath, 'r') as f:
                    self._config = yaml.safe_load(f) or {}
            else:
                self._config = {}
            return True
        except Exception as e:
            print(f"Failed to load config from {filepath}: {e}")
            return False
    
    def save_to_file(self, filepath: Optional[Union[str, Path]] = None) -> bool:
        """Save configuration to YAML file"""
        path = Path(filepath) if filepath else self._filepath
        if not path:
            return False
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'w') as f:
                yaml.safe_dump(self._config, f, default_flow_style=False, sort_keys=True)
            return True
        except Exception as e:
            print(f"Failed to save config to {path}: {e}")
            return False
    
    def get(self, key: str, default: Any = None) -> Any:
        """Get configuration value with dot notation (e.g., 'formation.kp_xy')"""
        keys = key.split('.')
        value = self._config
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        return value
    
    def set(self, key: str, value: Any) -> bool:
        """Set configuration value with dot notation"""
        if key in self._schemas:
            schema = self._schemas[key]
            if schema.min_value is not None and value < schema.min_value:
                raise ValueError(f"{key}: value {value} below minimum {schema.min_value}")
            if schema.max_value is not None and value > schema.max_value:
                raise ValueError(f"{key}: value {value} above maximum {schema.max_value}")
        
        keys = key.split('.')
        target = self._config
        for k in keys[:-1]:
            if k not in target:
                target[k] = {}
            target = target[k]
        target[keys[-1]] = value
        return True
    
    def has(self, key: str) -> bool:
        return self.get(key) is not None
    
    def get_keys_with_prefix(self, prefix: str) -> List[str]:
        """Get all keys starting with prefix"""
        result = []
        prefix_parts = prefix.split('.')
        
        def _collect(d: Dict, path: List[str]):
            for k, v in d.items():
                current_path = path + [k]
                full_key = '.'.join(current_path)
                if full_key.startswith(prefix):
                    result.append(full_key)
                if isinstance(v, dict):
                    _collect(v, current_path)
        
        _collect(self._config, [])
        return result
    
    def register_schema(self, schema: ConfigSchema) -> None:
        self._schemas[schema.name] = schema
    
    def validate_all(self) -> List[str]:
        """Validate all registered configs, return list of errors"""
        errors = []
        for name, schema in self._schemas.items():
            value = self.get(name)
            if schema.required and value is None:
                errors.append(f"Required config '{name}' is not set")
            if value is not None:
                if schema.min_value is not None and value < schema.min_value:
                    errors.append(f"'{name}': value {value} below minimum {schema.min_value}")
                if schema.max_value is not None and value > schema.max_value:
                    errors.append(f"'{name}': value {value} above maximum {schema.max_value}")
        return errors
    
    def reset_to_defaults(self) -> None:
        for schema in self._schemas.values():
            self.set(schema.name, schema.default_value)
    
    def to_yaml(self) -> str:
        return yaml.dump(self._config, default_flow_style=False, sort_keys=True)
    
    def from_yaml(self, yaml_str: str) -> bool:
        try:
            self._config = yaml.safe_load(yaml_str) or {}
            return True
        except Exception as e:
            print(f"Failed to parse YAML: {e}")
            return False
    
    def get_subconfig(self, prefix: str) -> Dict[str, Any]:
        """Get all config values under a prefix as flat dict"""
        result = {}
        for key in self.get_keys_with_prefix(prefix):
            value = self.get(key)
            if value is not None:
                # Remove prefix
                short_key = key[len(prefix):].lstrip('.')
                result[short_key] = value
        return result


# Parameter dataclasses (matching firmware params)

@dataclass
class FormationParams:
    kp_xy: float = 0.8
    ki_xy: float = 0.05
    kd_xy: float = 0.15
    kp_z: float = 1.0
    ki_z: float = 0.1
    kd_z: float = 0.2
    kp_yaw: float = 2.0
    max_vel_xy: float = 5.0
    max_vel_z: float = 2.0
    max_accel_xy: float = 3.0
    max_accel_z: float = 2.0
    formation_spacing: float = 5.0
    lateral_tolerance: float = 0.5
    vertical_tolerance: float = 0.3


@dataclass
class TaskAllocParams:
    auction_rate_hz: float = 1.0
    bid_timeout_s: float = 5.0
    consensus_threshold: float = 0.66
    capability_weight: float = 1.0
    distance_weight: float = 0.5
    energy_weight: float = 0.3


@dataclass
class AiPerceptionParams:
    obstacle_confidence_threshold: float = 0.7
    landing_confidence_threshold: float = 0.7
    fault_confidence_threshold: float = 0.7
    max_detections: int = 100
    nms_threshold: float = 0.45
    inference_batch_size: int = 1


@dataclass
class MeshParams:
    wifi_tx_power_dbm: float = 20.0
    link_timeout_s: float = 10.0
    store_forward_max_age_s: float = 30.0
    qos_update_interval_s: float = 1.0
    max_hops: int = 5


@dataclass
class SafetyParams:
    geofence_buffer_m: float = 10.0
    emergency_rtl_altitude_m: float = 50.0
    battery_critical_pct: float = 0.15
    battery_rtl_pct: float = 0.25
    estop_timeout_ms: int = 1000
    max_wind_mps: float = 15.0


@dataclass
class PlatformParams:
    motor_count: int = 4
    motor_kv: float = 850.0
    propeller_size_inch: float = 15.0
    max_thrust_n: float = 20.0
    battery_cells: int = 6
    battery_capacity_ah: float = 22.0
    mass_kg: float = 2.5


# Global config manager instance
_config_manager: Optional[ConfigManager] = None
_config_lock = threading.Lock()


def get_config_manager() -> ConfigManager:
    global _config_manager
    with _config_lock:
        if _config_manager is None:
            _config_manager = ConfigManager()
        return _config_manager


def load_config(filepath: Union[str, Path]) -> bool:
    return get_config_manager().load_from_file(filepath)


def save_config(filepath: Optional[Union[str, Path]] = None) -> bool:
    return get_config_manager().save_to_file(filepath)


def get_param(key: str, default: Any = None) -> Any:
    return get_config_manager().get(key, default)


def set_param(key: str, value: Any) -> bool:
    return get_config_manager().set(key, value)


# Register default schemas
def register_default_schemas():
    cm = get_config_manager()
    
    # Formation params
    for field_name, field_info in FormationParams.__dataclass_fields__.items():
        cm.register_schema(ConfigSchema(
            name=f"formation.{field_name}",
            description=f"Formation control {field_name}",
            default_value=field_info.default,
            min_value=0.0 if field_info.type in (float, int) else None,
        ))
    
    # Task alloc params
    for field_name, field_info in TaskAllocParams.__dataclass_fields__.items():
        cm.register_schema(ConfigSchema(
            name=f"task_alloc.{field_name}",
            description=f"Task allocation {field_name}",
            default_value=field_info.default,
        ))
    
    # AI perception params
    for field_name, field_info in AiPerceptionParams.__dataclass_fields__.items():
        cm.register_schema(ConfigSchema(
            name=f"ai_perception.{field_name}",
            description=f"AI perception {field_name}",
            default_value=field_info.default,
        ))
    
    # Mesh params
    for field_name, field_info in MeshParams.__dataclass_fields__.items():
        cm.register_schema(ConfigSchema(
            name=f"mesh.{field_name}",
            description=f"Mesh communication {field_name}",
            default_value=field_info.default,
        ))
    
    # Safety params
    for field_name, field_info in SafetyParams.__dataclass_fields__.items():
        cm.register_schema(ConfigSchema(
            name=f"safety.{field_name}",
            description=f"Safety {field_name}",
            default_value=field_info.default,
        ))
    
    # Platform params
    for field_name, field_info in PlatformParams.__dataclass_fields__.items():
        cm.register_schema(ConfigSchema(
            name=f"platform.{field_name}",
            description=f"Platform {field_name}",
            default_value=field_info.default,
        ))


# Convenience functions for parameter groups
def get_formation_params() -> FormationParams:
    cm = get_config_manager()
    return FormationParams(**cm.get_subconfig("formation"))


def get_task_alloc_params() -> TaskAllocParams:
    cm = get_config_manager()
    return TaskAllocParams(**cm.get_subconfig("task_alloc"))


def get_ai_perception_params() -> AiPerceptionParams:
    cm = get_config_manager()
    return AiPerceptionParams(**cm.get_subconfig("ai_perception"))


def get_mesh_params() -> MeshParams:
    cm = get_config_manager()
    return MeshParams(**cm.get_subconfig("mesh"))


def get_safety_params() -> SafetyParams:
    cm = get_config_manager()
    return SafetyParams(**cm.get_subconfig("safety"))


def get_platform_params() -> PlatformParams:
    cm = get_config_manager()
    return PlatformParams(**cm.get_subconfig("platform"))