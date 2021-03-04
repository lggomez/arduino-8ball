# arduino-8ball
Arduino implementation of an 8ball toy, using an sh1106 display and a MPU6050 accelerometer module

## Prototype:
Just ask your question and give it a little shake!
<p align="center">
  <img src="/prototype_build.jpeg" width="500">
</p>

## (Very) basic schematic:
<p align="center">
  <img src="/8ball_bb.png" width="300">
</p>

# v1

**(Update february 2021)**: After a redisign of the internals (and updates of the dependencies that make them more stable) this project is in a v1 state, after some months of years of lack of maintenance:

This can be mounted on any small box (provided that you cannot print a ball shaped container) and it is currently based on an Arduino Nano:

![20210303_232549](https://user-images.githubusercontent.com/6326271/109906737-cd2e8180-7c7f-11eb-8654-5d8ae8fbcb92.jpg)

It is also wired to a power switch and a 9v battery to make the device portable (and give it some additional mass for better feeling upon shaking)

![20210303_232811](https://user-images.githubusercontent.com/6326271/109906752-d4558f80-7c7f-11eb-9793-0cb5c10df224.jpg)

## Video demo

https://user-images.githubusercontent.com/6326271/109906586-85a7f580-7c7f-11eb-9a72-98b97e9639d9.mp4
