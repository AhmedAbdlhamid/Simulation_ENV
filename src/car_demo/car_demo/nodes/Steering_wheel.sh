
# !/bin/bash
echo "RERUN with sudo ./Steering_wheel.sh for permission "
sudo echo "
DefaultVendor=046d
DefaultProduct=c261
MessageEndpoint=01
ResponseEndpoint=01
TargetClass=0x03
MessageContent="0f00010142"">> /etc/usb_modeswitch.d/046d:c261

sudo usb_modeswitch -c /etc/usb_modeswitch.d/046d:c261






