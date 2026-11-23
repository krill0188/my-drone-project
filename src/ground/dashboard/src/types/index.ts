// Core domain types for the AI Swarm Dashboard

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface GeoCoordinate {
  lat: number;
  lon: number;
  alt?: number;
}

export interface Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

export interface EulerAngles {
  roll: number;
  pitch: number;
  yaw: number;
}

// Vehicle/Drone types
export interface DroneState {
  id: number;
  callsign: string;
  firmware_type: 'PX4' | 'ArduPilot';
  
  // Position & attitude
  position: GeoCoordinate;
  position_ned: Vector3;
  attitude: EulerAngles;
  quaternion: Quaternion;
  heading: number;
  ground_speed: number;
  vertical_speed: number;
  
  // State
  flight_mode: string;
  arming_state: 'ARMED' | 'DISARMED';
  nav_state: string;
  is_flying: boolean;
  is_leader: boolean;
  leader_id: number;
  
  // Battery
  battery_voltage: number;
  battery_current: number;
  battery_remaining: number;
  battery_consumed: number;
  cell_voltages: number[];
  cell_count: number;
  
  // GPS
  gps_satellites: number;
  gps_hdop: number;
  gps_fix_type: number;
  gps_healthy: boolean;
  
  // Estimator
  ekf_innovation: number;
  ekf_healthy: boolean;
  ekf_reset_count: number;
  
  // Communication
  link_quality: number;
  rssi_dbm: number;
  snr_db: number;
  latency_ms: number;
  last_heartbeat: number;
  messages_received: number;
  messages_lost: number;
  
  // AI/Perception
  active_detections: number;
  inference_time_ms: number;
  last_detection_class: string;
  last_detection_confidence: number;
  
  // Safety
  geofence_ok: boolean;
  estop_active: boolean;
  last_safety_event: string;
  
  // Formation
  formation_error: number;
  formation_target: GeoCoordinate;
  in_formation: boolean;
  
  // Timestamps
  last_update: string;
  first_seen: string;
}

export interface SwarmFormationState {
  leader_id: number;
  formation_type: 'line' | 'vee' | 'diamond' | 'grid' | 'custom';
  member_ids: number[];
  assigned_positions: Record<number, GeoCoordinate>;
  actual_positions: Record<number, GeoCoordinate>;
  position_errors: Record<number, number>;
  avg_formation_error: number;
  max_formation_error: number;
  formation_locked: boolean;
  formation_state: 'forming' | 'holding' | 'transitioning' | 'lost';
  last_update: string;
}

export interface MeshQoSMetrics {
  node_id: number;
  link_quality_to: Record<number, number>;
  rssi_dbm_to: Record<number, number>;
  latency_ms_to: Record<number, number>;
  packets_sent_to: Record<number, number>;
  packets_received_from: Record<number, number>;
  packets_lost_to: Record<number, number>;
  avg_link_quality: number;
  min_link_quality: number;
  active_links: number;
  total_neighbors: number;
  radio_type: 'WiFi6E' | '5G' | 'LoRa';
  channel: number;
  bandwidth_mhz: number;
  tx_power_dbm: number;
  last_update: string;
}

export interface SafetyEvent {
  timestamp: string;
  source_id: number;
  event_type: 'GEOFENCE_BREACH' | 'ESTOP' | 'COLLISION_WARNING' | 'LOW_BATTERY' | 'GPS_LOSS' | 'LINK_LOSS';
  severity: 'INFO' | 'WARN' | 'ERROR' | 'CRITICAL';
  description: string;
  details: Record<string, any>;
  acknowledged: boolean;
  acknowledged_time: string | null;
  acknowledged_by: number | null;
}

export interface AIDetection {
  timestamp: string;
  vehicle_id: number;
  class_name: string;
  class_id: number;
  confidence: number;
  position: GeoCoordinate;
  bounding_box?: [number, number, number, number]; // x, y, w, h normalized
  inference_time_ms: number;
}

export interface SwarmSummary {
  total_vehicles: number;
  armed_count: number;
  flying_count: number;
  leader_id: number;
  formation_type: string;
  formation_state: string;
  avg_formation_error: number;
  max_formation_error: number;
  unacknowledged_safety_events: number;
  critical_safety_events: number;
  avg_battery_pct: number;
  avg_link_quality: number;
  timestamp: string;
}

// API Response types
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
  timestamp: string;
}

export interface PaginatedResponse<T> {
  items: T[];
  total: number;
  page: number;
  page_size: number;
}

// WebSocket message types
export interface WSMessage<T = any> {
  type: string;
  payload: T;
  timestamp: string;
}

export interface WSVehicleUpdate extends WSMessage<DroneState> {
  type: 'vehicle_update';
}

export interface WSFormationUpdate extends WSMessage<SwarmFormationState> {
  type: 'formation_update';
}

export interface WSMeshQoSUpdate extends WSMessage<MeshQoSMetrics> {
  type: 'mesh_qos_update';
}

export interface WSSafetyEvent extends WSMessage<SafetyEvent> {
  type: 'safety_event';
}

export interface WSSwarmSummary extends WSMessage<SwarmSummary> {
  type: 'swarm_summary';
}

export type AnyWSMessage = 
  | WSVehicleUpdate 
  | WSFormationUpdate 
  | WSMeshQoSUpdate 
  | WSSafetyEvent 
  | WSSwarmSummary;

// Component prop types
export interface MapViewport {
  center: [number, number];
  zoom: number;
  bearing: number;
  pitch: number;
}

export interface ChartDataPoint {
  timestamp: string | number;
  value: number;
  label?: string;
}

export interface TimeSeriesData {
  label: string;
  data: ChartDataPoint[];
  color?: string;
  unit?: string;
}

// Filter types
export interface SafetyEventFilter {
  severity?: string[];
  source_ids?: number[];
  event_types?: string[];
  date_range?: [string, string];
  acknowledged?: boolean;
}

export interface LogFilter {
  vehicle_ids?: number[];
  date_range?: [string, string];
  log_types?: string[];
}

// Report types
export interface ReportOptions {
  format: 'html' | 'pdf' | 'json';
  include_coverage: boolean;
  include_formation: boolean;
  include_ai: boolean;
  include_qos: boolean;
  template: 'standard' | 'detailed' | 'executive' | 'regulatory';
}

export interface ReportMetadata {
  id: string;
  flight_log_id: string;
  generated_at: string;
  status: 'generating' | 'completed' | 'failed';
  format: string;
  template: string;
  file_path: string;
  file_size: number;
  summary: string;
}