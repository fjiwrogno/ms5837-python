## Arduino Test

arduino is used for quick test about the hardware itself.

## Hardware Configuration

- vin->Left pin beside the ref is 3.3v
- gnd->GND
- A4->SDA
- A5->SCL

## Software Configuration

First of all, make sure to have a usb-typeCc wire not only provide +5v but also can upload the code.

- board: arduino nano
- processor: ATMega328P
- port: choose the port that is detected by the pc

## Pipeline

 1. Compiling the code by clicking the ✓ which means verify.
 2. Click the "uploading" button to upload the code into arduino board.
 3. Run "serial-plotter" or "serial-monitor" to check the output of sensor.
