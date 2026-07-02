# Write one line on the driver board's OLED ({"T":3,"lineNum":n,"Text":s}).
# Send line=255 (any text) to restore the board's default screen ({"T":-3}).
uint8 line    # 0..3 = OLED line number; 255 = reset to default screen
string text   # the text to show (board truncates to its line width)
