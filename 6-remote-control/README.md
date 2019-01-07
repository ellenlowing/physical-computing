# Quest6 Navigating a Car

Author: Ellen Lo, Lin Ma, Biyao Liang, 2018-12-14

## Summary

In this Quest, we integrated Webcam, Beacon receiver, and server control to make a car that we can remotely control. We have a html web page to display the webcam view to see the surrounding of the car. The car receives the directional control from the html webpage using HTTPD Module. The html website uses "GET" and "POST" request to send the directional command to the esp32 to control the motor through the server on the Raspberry Pi. When click the "forward", "backward", "left", and "right" button on the webpage, the car will go to the corresponding direction one step. Besides, the ultrasound on the car detects the distance between a obstacle and the front of a car. It this distance it greater then 40 cm the car will not move forward to avoid collision.      

## Evaluation Criteria

 - Car	is	controlled	remotely	from	DDNS	URL	and
drives	successfully	including	L,	R,	F,	R	controls.

 - Each	beacon	is	visited	by	car	and	each	message
fragement	decoded	in	program.

 - Video	incorporated	in	single	browser	frame	with
controls.

 - Collision	avoidance	used;	no	collisions

 - Catenated	message	formed	in	program	and
decoded	automatically	in	browser	showing
hidden	message.


## Solution Design

  Webcam connects to the node.js server on the raspberry pi. The car's motor and ultrasound sensor is connected to the raspberry pi through HTTPD. The network is hosted through the LINKSYS router.

## Sketches and Photos

  - Top view
  ![top](https://user-images.githubusercontent.com/24300238/50047656-e55a5c00-0087-11e9-94df-1c52004a19f0.JPG)

 - Front view
 ![ffront](https://user-images.githubusercontent.com/24300238/50047772-1c317180-008a-11e9-8c4c-a851f9b60328.JPG)

 - Side view
 ![side](https://user-images.githubusercontent.com/24300238/50047664-1a66ae80-0088-11e9-8c95-38ec71d15ad6.JPG)


## Links

 - http://whizzer.bu.edu/team-quests/primary/remote-control  


## Modules, Tools, Source Used in Solution

- Ultrasound

- PWM

- node.js

- HTML


## Supporting Artifacts
