# Underwater Housing Leak Detector

Monitors the underpressure of your underwater housing and notifies you of
any leaks.

## Features

The leak detector provides the following features:

- Optical and acoustical alarm. Red and green LED to indicate the current
  health. Piezo buzzer to get the user's attention.
- Powered by just one CR2032 coin cell.
- Very long battery life, works down to 1.8V cell voltage.
- Battery monitoring. Indicates low-battery during idle mode.
- Hackable design using a Microchip ATtiny microcontroller
- MEMS pressure sensor. Supports two different devices: Bosch BMP581 and
  STMicro LPS22HB.
- Demo mode. Test any changes by a simulated sensor.
- Optical UART debug mode. The red LED can be used as a serial port. The
  receiver consists of a photo transistor and a bipolar transistor in a
  darlington configuration and an UART-to-USB adapter.
- Different modes selectable by the reset button.
- Extremely small.
- Open Hardware and Open Firmware schematics and layout as well as firmware
  sources are free for everyone.

## User's manual

TBD

## Mode of operation

Most functionality revolves around the microcontroller polling the pressure
sensor at a low frequency and then go (back) to sleep. The goal is to be in
sleep mode most of the time to save precious battery power. The
microcontroller runs with a system clock of 1MHz. The assumption is that a
slower clock frequency will preserve more power. But it is not clear whether
this is true; it might as well save more power if the microcontroller can
go back to sleep faster. One would need to do power consumption
measurements to evaluate which is better.

There are basically two different modes:
- Idle mode. The detector waits for a pressure drop to go into supervisor
  mode. In this mode, the pressure is polled every two seconds.
- Supervisor mode. The detector monitors the pressure at a faster rate and
  drives the dual-color LED/buzzer to show the state to the user.

Internally, the firmware consists of a [state
machine](firmware/main.c#L49). The state maps to the external indications
at any time. E.g. if the program is in state `ALARM`, the LED is flashing
red and the buzzer is enabled.

The firmware is able to count the number of reset button presses. Depending
on this the firmware enters different modes:
- Normal mode
- Debug mode. Red LED is used to transmit debug information.
- Demo mode. Instead of reading the real pressure, predefined measurements
  are replayed.
- Demo mode with debug. Combines demo and debug mode.
- Power-off mode. Turn off periodic watchdog interrupt. Only an external
  reset can wake up the microcontroller.

### ATtiny hardware block usage

The *Timer/Counter0* is used to stimulate the piezo buzzer with a 2kHz
frequency.

The *Timer/Counter1* is either used in PWM mode to dim the LEDs or to drive
the software UART. Both are mutually exclusive and the startup will choose
between these two.

The *Universal Serial Interface* is used for the I2C communication with the
pressure sensor. There is no receiving ACK handling (yet).

The *Watchdog timer* is used in interrupt mode as a wake up source. It will
wake up the microcontroller either every 250ms or every 2s, depending on
the mode. Additionally, it is used for timekeeping by counting ticks.

The *Analog to Digital Converter* is used in reverse mode (AREF is VCC and
the measurement is the internal band gap voltage) to measure the battery
voltage.

The *Analog Comparator` is unused and disabled on start-up.

The hardware is prepared to use interrupts via the *Pin Change Interrupt`
but it is unused. The sensor supports to constantly measure the pressure
and generate an interrupt if it is outside a predefined window. But it is
very limited in this regard, thus it not used for the time being.

### Supported pressure sensors

The firmware supports two different pressure sensors:
- [Bosch Sensortec
  BMP581](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp581/)
- [STMicroelectronics
  LPS22HB](https://www.st.com/en/mems-and-sensors/lps22hb.html)

They share the same footprint and can be used with the same PCB. The BMP581
is more advanced regarding its features, but most of them are unused. One
major drawback is the package and metal can of the sensor. There is a tiny
hole in the can which sometimes clogs and the sensor will measure bogus
values. Also, it isn't easy to convert the raw value of the sensor to
millibars. It will need division routines which takes up a lot of code
space.

The LPS22HB is a rather old sensor and consumes more power. It's package
doesn't have any holes but an exposed silicon area to sense the pressure.
Therefore, it is not prone to clogging like the BMP581. Also the register
interface seems to be more thought through. There are also successors of
the LPS22HB which need less power. Unfortunately, they aren't available at
the moment.

### Resource usage

TBD

## Programming the microcontroller

The board features a [Tag-Connect](https://www.tag-connect.com) with the
standard 6pin header AVR pinout. The microcontroller can be programmed
using the simple SPI based programming protocol. There is no need for
high-voltage programming.

## Schematics

The schematics are available [here](hardware/leak-detector.pdf).

## Bill Of Materials

| Ref | Qty | Part | Footprint |
| --- | --- | --- | --- |
| R1 | 1 | 33Ω Resistor | 0603 |
| R2, R7 | 2 | 1kΩ Resistor | 0603 |
| R3 | 1 | 10Ω Resistor | 0603 |
| R4, R5 | 2 | 100Ω Resistor | 0603 |
| R6, R8, R9 | 3 | 10kΩ Resistor | 0603 |
| D1 | 1 | Dual-color LED | - |
| U1 | 1 | BMP581/LPS22HB pressure sensor | LGA-10 |
| U2 | 1 | ATtiny25V | SOIC-8 |
| SW2 | 1 | PTS810 Push Button | - |
| BT1 | 1 | CR2032 Coin Cell Holder | - |
| BZ1 | 1 | Visaton 3580 Piezo Buzzer | - |

## Kits and PCBs

If you need a PCB, a kit or an assembled and pre-programmed board, contact
me at <uwh-leak-detector@walle.cc>.

## License

The schematics and layout is licensed under the CERN-OHL-S v2. The firmware
source is licensed under the GNU General Public License v3.0.
