# JetsonNanoUart

This is a C++ class to send and receive over UART on the Nvidia Jetson Nano.   
I'm using this to communicate with an Arduino Nano.  
The wiring is simple :
- TX -> RX
- RX <- TX
- GND -> GND


On the Nvidia Jetson Nano you connect on 8 and 10 on the J41 pin header and a GND.


The original post :
https://devtalk.nvidia.com/default/topic/1057441/jetson-nano/jetson-nano-uart-c-c-example/post/5395530/#5395530
