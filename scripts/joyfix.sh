# Temporary script to fix gamepad button mapping. Source:
# https://steamcommunity.com/app/286690/discussions/1/610575007206589892/
# How to install:
# $ sudo add-apt-repository ppa:grumbel/ppa
# $ sudo apt-get update
# $ sudo apt-get install -y xboxdrv hwinfo
# How to use:
# 1. Connect your gamepad via bluetooth
# 2. Open new tmux (or whatever) session
# 3. Run "scripts/joyfix.sh", detach
# 4. Verify you now have device /dev/input/js1
# 5. Run gamepad node:
# $ ros2 run joy_linux joy_linux_node --ros-args -p dev:="/dev/inupt/js1

xboxdrv \
   --evdev /dev/input/event6                        \
   --evdev-absmap ABS_X=x1,ABS_Y=y1                 \
   --evdev-absmap ABS_Z=x2,ABS_RZ=y2                \
   --evdev-absmap ABS_HAT0X=dpad_x,ABS_HAT0Y=dpad_y \
   --evdev-keymap BTN_A=x,BTN_B=a                   \
   --evdev-keymap BTN_C=b,BTN_X=y                   \
   --evdev-keymap BTN_Y=lb,BTN_Z=rb                 \
   --evdev-keymap BTN_TL=lt,BTN_TR=rt               \
   --evdev-keymap BTN_SELECT=tl,BTN_START=tr        \
   --evdev-keymap BTN_TL2=back,BTN_TR2=start        \
   --evdev-keymap BTN_MODE=guide                    \
   --axismap -y1=y1,-y2=y2                          \
   --mimic-xpad                                     \
   --silent
