# iae_robot
On a Pi 5 running bookworm

sudo apt update
sudo apt upgrade

GPIO motor control Library
sudo apt install python3-gpiozero

Servo Driver Libraries
pip3 install adafruit-blinka
pip3 install adafruit-circuitpython-pca9685
pip3 install adafruit-circuitpython-motor

Enbale I2C for Servo Hat
sudo raspi-config
Interface Options
I2C = Enable
ssh =Enable