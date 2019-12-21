# miam_utils

This library is a very generic C++ library for handling all low-level activities: sensor drivers, PIDs, kinematics...

## Compiling this code

This code can be either compiled for the local host, or cross-compiled for an arm platform with ```arm-linux-gnueabihf-g++```.
To compile this code:
```
	cmake /path/to/miam_utils -DCMAKE_INSTALL_PREFIX=<installPath> -DCROSS_COMPILE=<ON,OFF>
	make
	make install
```

Once installed, this library can be imported using pkg-config. During install, the pkg-config file `miam_utils.pc` is installed in
`<installPath>/lib/pkgconfig` : this path must thus be added to the `PKG_CONFIG_PATH` variable. To do so, open your `.bashrc` file

```
gedit ~/.bashrc
```

and append the following line :

```
export PKG_CONFIG_PATH=<installPath>/lib/pkgconfig
```

Remember to `source ~/.bashrc` before use.

When cross-compiling, the library is called ```miam_utils_arm``` instead.


There is a doxygen documentation for this library in `BBBEurobot/doc`. Simply run `make_documentation.sh` to generate it.

## Running code of the Beaglebone.

When running code on a Beaglebone Black, the GPIOs and serial ports need to be enabled. This is done using a device tree overlay (DTO) file.
A DTO file for BBB-Eurobot, enabeling all the ports exposed on the PCB (plus UART4, which is needed to talk to the servo driver - it wasn't planned
on the PCB but a wire can easily be solder to make this connection. Note that you need a 3.3V to 5V converter).

This file is called Eurobot-00A0.dts, and can be found in the library folder (BBB-Eurobot). To use it, copy this file to the Beaglebone,
then compile it and copy it to /lib/firmware:

```
	dtc -O dtb -o Eurobot-00A0.dtbo -b 00 -@ Eurobot-00A0.dts
	cp Eurobot-00A0.dtbo /lib/firmware
```

Now doing ```echo Eurobot > /sys/devices/bone_capemgr.9/slots``` enables all these ports for the duration of the current boot (this is done internally by BBB-Eurobot).
