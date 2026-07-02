# Typed rover telemetry, republished from the ESP32 "General Driver Board" feedback
# frame ({"T":1001, "L","R","r","p","v"}). This is the stable, swappable contract the
# dashboard and any consumer subscribe to -- no one downstream parses raw board JSON.
std_msgs/Header header

float32 left_speed    # m/s, left tread (board "L", encoder-derived)
float32 right_speed   # m/s, right tread (board "R", encoder-derived)
float32 roll          # deg, board-reported roll (board "r"); UNRELIABLE on stock
float32 pitch         # deg, board-reported pitch (board "p"); firmware (see docs/FIRMWARE.md)
float32 yaw           # deg, board-reported yaw (board "y"); 0.0 on firmware that lacks it
float32 voltage       # V, battery (board "v")
float32 temp          # degC-ish, ESP32 die temp (board "temp"); 0.0 when absent
