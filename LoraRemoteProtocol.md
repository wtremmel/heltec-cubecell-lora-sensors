# Lora Data Channels
* 01 - temperature
* 02 - humidity
* 03 - barometric pressure
* 04 - luminosity
* 05 - voltage of device
* 06 - rain
* 07 - memory free (debug)
* 08 - GPS location
* 10-14 - rtc epoch (debug)
* 20-23 - adc1150 values


# Lora Remote Control

Definition of commands to be sent to my Lora based sensor nodes. To keep messages short, byte definitions are unsigned

## Byte 0 - select command group
* 0x00 - system commands
* 0x01 - sensor commands
* 0x02 - sending commands


## Commands by group
### System commands - (Byte 0 == 0x00)
* `0x01` - Power save (default on). Argument switches it on or off.
  * `0x01 0x00` power save off
  * `0x01 0x01` power safe on (default)
  * `0x01 0x10` hibernation on
  * `0x01 0x11` hibernation off
  * `0x01 0xff` drain battery (implementation specific)
* `0x02` - Delay time. Argument is delay in seconds times 5. So `0x02 0x0c` is 60 seconds delay/sleep between measurements. `0x02 0x00` is variable delay (default)
* `0x03` - LED usage.
  * `0x03 0x00` - LED off (default)
  * `0x03 0x01` - LED on
  * `0x03 0x02 0xrr 0xgg 0xbb` - for RGB LEDs, set color to rrggbb
  * `0x03 0xff` - LED dynamic - shows status
* `0x04 0x00 0x00 0x00 0x00`  - send current time to the node
* `0x05 <hours> <minutes> <seconds>` - sleep now for some time (one byte each)
* `0x06 0x00000000` - sleep now for seconds
* `0x07` - garbage collection (free mem). Also sends message with free memory in slot 7.
* `0x08` - set timing parameters.
  * `0x08 0x00` - set all values to default
  * `0x08 0x01 mm` - set minimum cycle time to mm minutes
  * `0x08 0x02 mm` - set maximum cycle time to mm minutes (00 08 02 3c)
  * `0x08 0x03 mm` - set hibernation sleep time to mm minutes (00 08 03 78)
  * `0x08 0x11 vv` - set min voltage to v.v V (00 08 11 25) 3.7V
  * `0x08 0x12 vv` - set max voltage to v.v V (00 08 12 2A) 4.2V
  * `0x08 0x13 vv` - set shutdown voltage to v.v V (00 08 13 25) 3.7V
  * `0x08 0x14 vv` - set restart  voltage to v.v V (00 08 14 25) 3.7V
  * `0x08 0xff <minCycle> <maxCycle> <hibernationSleep> <minVoltage> <maxVoltage> <shutdownV> <restartV>` - set all values in one message
* `0x09` - rescan i2c bus
* `0xff` - reboot. Reboots the node (if possible)

### Sensor commands - (Byte 0 == 0x01)
* `0x00` - disable all sensor readings
* `0x01` - enable sensor readings
* `0x1x` - configure sensor x (see below for list)
  * `0x11` - Configure BME280
    * `0x11 0x01` - stop temperature reading
    * `0x11 0x02` - stop humidity reading
    * `0x11 0x04` - stop pressure reading
    * `0x11 0x07` - stop all readings (combined three bits together)
    * `0x11 0xff` or `0x11 0x00` - restart all readings
  * `0x12` - Configure TSL2561
    * `0x12 0x01` - stop luminosity reading
    * `0x12 0xff` or `0x12 0x00` - restart reading

### Sending commands - (Byte 0 == 0x02)
* `0x01` - switch sending type
  * `0x01 0x01` - send unconfirmed
  * `0x01 0x02` - send confirmed
  * `0x01 0x00` - set default (device dependent)
* `0x20` - send all queued
* `0x21` - send all in delete queue

### Display commands - (Byte 0 == 0x03)
* `0x00` - display off
* `0x01` - display default content
* `0x02 <message>` - display message
