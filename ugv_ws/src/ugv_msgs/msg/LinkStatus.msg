# Health of the Pi <-> ESP32 serial link, published by ugv_driver at ~1Hz.
# The dashboard's "driver" pill reads this; it is also the place to look when
# telemetry goes quiet (reconnects/rearms counting up = the link is flapping).
std_msgs/Header header

bool connected          # serial fd open right now
bool estop              # software e-stop latch is engaged (cmd_vel ignored)
float32 telemetry_age   # s since the last {"T":1001} frame (-1 = never seen)
uint32 reconnects       # times the fd was closed + re-opened since start
uint32 rearms           # times the feedback stream was re-armed since start
