export DISPLAY=:0
Xvfb -screen 0 1600x1200x24+32 &
fluxbox &
x11vnc &