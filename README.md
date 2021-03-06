# Long-Range-Arduino-Doorbell

The goal of this project was to create a cheap, very reliable and efficient long-range wireless doorbell.

- Price: < $10
- Range: Up to 1000 meters
- Power consumption (Idle): < 1µA @ 3V (at 3.27V I measured 193nA)

## Features

- The reliability is ensured by two-way communication between the transmitter and receiver. (If the receiver fails to receive a signal due to interference, the transmitter sends another signal until it verifies that it has arrived at the receiver.)
- The transmitter (which runs on two AAA batteries) reports its remaining battery capacity back to the receiver which can then warn you once it gets too low.
- The receiver uses a simple module that plays MP3 files from a micro SD card, so you can make the doorbell sound any way you want.

## Required parts

- 1x Arduino Pro Mini (3.3v 8Mhz edition)
- 1x Another Arduino (doesn't matter which one)
- 2x Button
- 2x D-SUN CC1101 module
- 1x DF-Player module
- 1x micro SD card (any size)
- 2x AAA battery (alkaline)
- 1x speaker (I will update this once I've settled on a specific one)
- 1x amplifier (should match power and impedence of speaker)

## How does it work

```
[Battery]--[Arduino]--[CC1101]-)))))))))))   (((((((((((-[CC1101]--[Arduino]--[5v power]
               |                                                       |
            [Button]                                               [speaker]
```

- You press the button
- First Arduino wakes up out of power-saving (sleep) mode
- Signal is sent to the second Arduino
- Second Arduino plays a doorbell sound
- Second Arduino sends a response to the first Arduino
- First Arduino receives the response and goes back to deep sleep
- (If no response is received the first Arduino tries again...)

## Schematics

Coming soon...

## How assemble

(It is highly recommended to desolder the LED and voltage regulator from the Arduino Pro Mini for minimal power consumption <1µA.)

Connection between D-SUN CC1101 modules and Arduinos:

```
GDO0 -> 6
MISO -> 12
MOSI -> 11
GND -> GND
CSN -> 10
SCK -> 13
VCC -> 3v
```

Connection between DF-Player and Arduino Uno (or whatever you decided on using):

```
VCC -> VCC
GND -> GND
TX -> 9
RX -> 8
```

Connection between button and Arduino Pro Mini:

```
Pin A -> 2
Pin B -> GND
```

## Credits

Big thanks to [LSatan](https://github.com/LSatan) for his [CC1101 library](https://github.com/LSatan/SmartRC-CC1101-Driver-Lib) and [for implementing two-way communication and CRC checksum verification on request and even proving complete examples](https://github.com/LSatan/SmartRC-CC1101-Driver-Lib/issues/43). This project is very much based on the work of LSatan. So leave a star on his repository and consider [donating to him](https://github.com/LSatan/SmartRC-CC1101-Driver-Lib#donation).
