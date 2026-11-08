# Captured UART line with source-side timestamp.
# direction: 0 = RX (MCU->host), 1 = TX (host->MCU)
uint8 DIRECTION_RX=0
uint8 DIRECTION_TX=1
uint64 host_mono_ns
builtin_interfaces/Time host_ros_stamp
uint64 capture_seq
string producer_epoch
uint8 direction
string line
bool tx_ok
