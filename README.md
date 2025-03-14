# MicroPython GPIO Driver

This project provides a simple GPIO driver for MicroPython, allowing you to easily control digital, analog, and PWM pins on your microcontroller.

## Files

- `boot.py`: Entry point for the MicroPython script.
- `gpio.py`: Contains the `GPIO` class for handling different types of pins.
- `main.py`: Example usage of the `GPIO` class.

## Usage

### GPIO Class

The `GPIO` class allows you to initialize and control GPIO pins. It supports digital, analog, and PWM pins.

#### Initialization

```python
from gpio import GPIO

# Digital pin as output
led = GPIO(14, pinType="digital", mode="OUT")

# Analog pin
adc = GPIO(35, pinType="analog")

# PWM pin
pwm_led = GPIO(25, pinType="pwm")
```

#### Methods

- `flash(t=1)`: Flash the digital output pin for `t` seconds.
- `read()`: Read the value from the pin.
- `write(state=None, dutyCycle=65535)`: Write a value to the pin.
- `toggle()`: Toggle the digital output pin.

### Example

The `main.py` file provides an example of how to use the `GPIO` class.

```python
from gpio import GPIO
from time import sleep

def main() -> None:
    adc = GPIO(35, pinType="analog")
    digpin = GPIO(12, pinType="digital", mode="IN")
    led = GPIO(14, pinType="pwm")
    led2 = GPIO(4, pinType="digital", mode="OUT")

    print(adc.read())
    print(digpin.read())
    led.write(dutyCycle=10)

    while True:
        led2.toggle()
        sleep(1)
        led2.toggle()
        sleep(1)

if __name__ == "__main__":
    main()
```

## License

This project is licensed under the MIT License.