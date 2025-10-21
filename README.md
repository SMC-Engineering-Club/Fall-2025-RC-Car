# Fall-2025-RC-Car
Resources, Documentation, and Part Lists for the Fall 2025 RC Car Project.


## Wiring "Diagram"

ESC -> FT 40amp ESC w/ XT60 

Motor -> FT 2814/1100KV Motor 

Servo -> FT20G 

Arduino -> Arduino Uno R4 Wifi 

Battery -> Tattu TAA23004S75X6 14.8V LiPo 



3-wire connector from ESC -> yellow into pin 9 and through a 10kΩ resistor to GND, red into 5V rail of breadboard, brown into GND rail of breadboard. 

3-wire connector from Servo -> yellow into pin 10 through a 220Ω resistor, red into 5V rail of breadboard, brown into GND rail of breadboard. 

~470 µF capactior across +5V and GND near the servo to tame current spikes. 

Arduino -> Pin 9 for ESC, Pin 10 for Steering Servo, 5V onto 5V rail of breadboard, GND onto GND rail of breadboard. (Once the circuit is connected to power via the XT60 from battery, do *not* connect the Arduino to any external power source including USB or Barrel Jack.) 

3-wire 3.5mm bullet connector from motor plugs into the 3-wire bullet connector from ESC. Any order is fine to test. No worry of shorts. Ensure to flip them around if the positive throttle spins reverse.
  

Lastly connect the XT60 on the ESC to the LiPo battery. This should power the motor, the servo, and the arduino all together. 







