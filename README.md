# iae_robot
On a Pi 5 running bookworm
```bash
sudo apt update
sudo apt upgrade
```
GPIO motor control Library
```bash
sudo apt install python3-gpiozero
```
Servo Driver Libraries
```bash
pip3 install adafruit-blinka
pip3 install adafruit-circuitpython-pca9685
pip3 install adafruit-circuitpython-motor
```
Enbale I2C for Servo Hat
```bash
sudo raspi-config
```
Interface Options
I2C = Enable
ssh =Enable

Using a Controller:
To use a controller, first you must conenct the device via bluetooth or usb.
Then
```bash
sudo apt install python3-pygame joystick
```
You can check whether the controller is detected by:
```bash
ls /dev/input
```
If js0 and js1 and new items appear, the controller is detected
