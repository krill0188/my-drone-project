# CommQoS.msg
# Mesh link quality metrics
uint64 timestamp_us
uint16 src_id
uint16 dst_id
uint8 link_type                        # 0=WIFI_6E, 1=FIVE_G_NR, 2=LORA, 3=MAVLINK_SERIAL
LinkQoS qos
uint16[] route                         # Multi-hop path (empty if direct)
uint8 status                           # 0=UP, 1=DEGRADED, 2=DOWN, 3=STORE_FORWARD
uint32 sequence

---
# LinkQoS
float32 latency_ms_p50
float32 latency_ms_p99
float32 packet_loss_pct
float32 bandwidth_mbps
float32 rssi_dbm
float32 snr_db
uint32 tx_bytes
uint32 rx_bytes
uint32 tx_errors
uint32 rx_errors