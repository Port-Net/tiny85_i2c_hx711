Library to use ATTiny85 as I2C slave to get measurements from HX711

Thanks for the developer of TinyWire and usiTwiSlave

The chip controlled like standard I2C devices by register access.
Its possible to measure only one channel with ~10 Hz or switch between the two channels wich recuces the sampling rate to about 1 Hz. The scale registers get only updated if the corresponding channel is enabled.
Support for therate pin is build in but not used because the used board provide no access to it.
There is support to use Timonel bootloader to been able to upgrade the software in system via I2C. Therefore the Timonel command for get version and switch-to-app (in this case switch to bootloader) are supported.

Config register 0x00, len 2: <br>
  byte 1:<br>
  | reboot | write EE | Rate | cont read | Channel B mode | Channel A mode |<br>
    Channel A Mode (Bit 0, 1): 0 -> off, 1 -> gain 128, 2 -> gain 32<br>
    Channel B Mode (Bit 2): 0 -> off, 1 -> gain 32<br>
    Cont Read (Bit 3, 4): 0 -> read reg, 1 -> scale1 + scale 2, 2 -> continue over all<br>
    Rate (Bit 5): 0 -> 10SPS, 1 -> 80SPS (not connected)<br>
    Write EE (Bit 6): 1 -> write config, tara, i2c address to EEPROM<br>
    Reboot (Bit 7): 1 -> reboot<br>
  byte 2: <br>
    averaging counter<br>
    
i2c_addr| 0x01, len 1<br>
tara A | 0x02, len 3<br>
tara B | 0x03, len 3<br>
scale A| 0x04, len 3<br>
scale B| 0x05, len 3<br>
avg scale A| 0x06, len 3<br>
avg scale B| 0x07, len 3<br>
raw scale A| 0x08, len 3<br>
raw scale B| 0x09, len 3<br>
<br>
To change the I2C address write reg 1 and then set "Write EE" to one to store config. The address is activated on next reboot. On store config and tara is saved.
