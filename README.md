# AI Robot for fire detection and extinguish
<b>B.Sc final project. Developing an autonomic AI robot for fire detection - using 2 microcontrollers for tasks separation (NVIDIA Jetson Nano for image processing and STM32 Cortex M4 for mechanics and sensors control).</b>


#### <ins>Final robot</ins>
![alt text](https://github.com/pawelgates/AI-Robot-for-fire-detection/blob/main/PICS/robot.JPG)

#### <ins>Fire Detection</ins>
NVIDEA Jetson Nano 2GB was used for Image Processing. We used SSD-Mobilenet-V2 with custom made FIRE DATASET for candle flames detection. When the object is detected, Jetson provides the "X" coordinate of the object for the STM32 microcontroller via UART communication. Robot will turn it self until the object will be detected in the middle of the frame. While there is no object, Jetson will send "XXXX" for the STM32.

![alt text](https://github.com/pawelgates/AI-Robot-for-fire-detection/blob/main/PICS/fire-detection.png)
![alt text](https://github.com/pawelgates/AI-Robot-for-fire-detection/blob/main/PICS/IMG_5235.JPG)
