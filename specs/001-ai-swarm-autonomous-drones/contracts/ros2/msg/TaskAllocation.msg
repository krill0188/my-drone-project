# TaskAllocation.msg
# CBBA task allocation messages
uint64 timestamp_us
uint16 drone_id
uint8[16] task_id                      # UUID bytes
float32 cost                           # Lower = better
CapabilityBitmap capabilities
uint32 sequence

---
# CapabilityBitmap
bool has_rtk_gps
bool has_stereo_camera
bool has_lidar_2d
bool has_lidar_3d
bool has_uwb
bool has_5g_modem
bool has_ai_compute
uint16 max_flight_time_s
float32 max_payload_kg
uint8 comm_tier_bitmap                 # bit0=WiFi6E, bit1=5G, bit2=LoRa