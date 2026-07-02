# Headlight / aux-LED control for the ESP32 "General Driver Board".
# Maps 1:1 to the board's {"T":132,"IO4":<0-255>,"IO5":<0-255>} command
# (IO4/IO5 are the two switched 12V LED channels on the UGV Rover).
uint8 io4   # PWM 0 (off) .. 255 (full)
uint8 io5   # PWM 0 (off) .. 255 (full)
