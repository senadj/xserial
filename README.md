# xserial
## _Simple Arduino X-Plane serial bridge (UART plugin)_

Developed and tested on Linux debian 11 (Bullseye)
using Boost Async serial-port:
[https://github.com/fedetft/serial-port ](https://github.com/fedetft/serial-port) 

Usage (you need to build the plugin first):
1. Upload **xserial.ino** sketch to Arduino
2. Put **xserial.xpl** and **xserial.ini** files in X-Plane plugins directory ( X_PLANE_INSTALL_DIR/Resources/plugins/ )
3. Set correct serial port/baudrate in xserial.ini file and make other desireable edits

After X-Plane is run, you can watch LED (pin 13) on Arduino turn on and off when moving gear handle up and down (on Cirrus you don't need to be in the air to move the handle).
If you connect servo motor to pin 6, it will move when you move flaps up or down.
Rudder, aileron and elevator will wiggle according to Analog pins readouts (try touching those pins with finger)
If you bring pin 11 LOW (connect it to ground), you will trigger gear down command (you need to be in the air to test this).
