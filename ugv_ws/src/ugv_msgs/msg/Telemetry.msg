# Typed rover telemetry, republished from the ESP32 "General Driver Board" feedback
# frame ({"T":1001, "L","R","r","p","v"}). This is the stable, swappable contract the
# dashboard and any consumer subscribe to -- no one downstream parses raw board JSON.
std_msgs/Header header

float32 left_speed    # m/s, left tread (board "L")
float32 right_speed   # m/s, right tread (board "R")
float32 roll          # board-reported roll (board "r"); units per firmware
float32 pitch         # board-reported pitch (board "p"); units per firmware
float32 voltage       # V, battery (board "v")
