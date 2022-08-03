**CSE522-RTES-Assignment-3**
Name: Ravi Shankar
ASU Id: 1220325912

CoAP Server
###########

Overview
********
This is an application program that builds a COAP server using Zephyr’s
network stack and COAP library for demostrationg the operations and 
communications of an IOT device and it uses chrome extension, Copper
for Chrome(Cu4Cr) as COAP user-agent, to interact with IOT devices.
The board is connected to the computer using Ethernet cable. A static 
address is set for the board. The devices are coap resources and a number
of COAP methods can be invoked on the resources to achieve either sensor
and actuator actions. The hcsr04 resources however have been created as COAP
observers who observe changes to certain important hscr04 sensor values.   

Tasks implemented
********************
1. The HC-SR04 senses the distance and reports the distance measure
in inches. A Kalman filter can be used to reduce any measurement
noise of HC-SR04 sensors.
2. A zephyr application that serves as a COAP server is devloped using
Zephyr’s network stack and COAP library. The server enables the following 
resources: 
a) /sensor/hcsr_i , i=0 and 1 for two HC-SR04 sensors. It uses get method 
request to retrieve the current distnace measure. A notification will be
sent when there is a change larger than 0.5 inches is distance measurement.
b) /sensor/period.  The put request method is used to define the sampling
period of the HC-SR04 sensor measurement.
c) /led/led_n  , n=r,g,b. It uses put method request to turn on or off the
led by setting it to 1 or 0 respectively. It uses get method request to 
receive the current led status.
d) .well-known/core interface for resource discovery. 
3.Generate the patch file:
diff –rauN --exclude=samples /(original zephyr tree) /(modified zephyr tree) > patch_project

Building and running
***********************
Note: COAP port number: 5683
      Device Ip address: 192.0.2.10 (prj.conf)
      Set a static host ip
1. Unzip RTES-Shankar-R_03.zip 
2. cd /RTES-Shankar-R_03/project_3, run  west build -b mimxrt1050_evk --pristine
3. Run west flash
4. Assign a static ip to the host
5. Ping the device ip to check if connection is established or 
use net ping from device to ping host.If ping is successful then
server is connected via Ethernet
6. Open putty and select port /dev/ttyACM0 and enter baud rate 115200 to see the serial debug logs of the server.

