# MicroPython GPIO Driver

This project provides a simple GPIO driver for MicroPython, allowing you to easily control digital, analog, and PWM pins on your microcontroller.

## Files

- `boot.py`: Entry point for the MicroPython script.
- `gpio.py`: Contains the `GPIO`, `LED`, and `Servo` classes for handling different types of pins.
- `main.py`: Example usage of the `GPIO`, `LED`, and `Servo` classes.

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

### LED Class

The `LED` class allows you to control an LED with digital or PWM pins.

#### Initialization

```python
from gpio import LED

# Digital LED
led = LED(14, pinType="digital")

# PWM LED
pwm_led = LED(25, pinType="pwm")
```

#### Methods

- `flash(t=1)`: Flash the LED for `t` seconds.
- `blink(n, t=0.15)`: Blink the LED `n` times with `t` seconds between blinks.
- `fadeIn()`: Gradually increase the brightness of the LED.
- `fadeOut()`: Gradually decrease the brightness of the LED.
- `morsecode(msg="SOS")`: Flash the LED in Morse code for the given message.

### Servo Class

The `Servo` class allows you to control a servo motor with PWM pins.

#### Initialization

```python
from gpio import Servo

# Servo motor
servo = Servo(16)
```

#### Methods

- `move(targetAngle)`: Move the servo to the target angle.
- `reset()`: Reset the servo to the 0 degree position.
- `mid()`: Move the servo to the 90 degree position.
- `max()`: Move the servo to the max degree position.

### Example

The `main.py` file provides an example of how to use the `GPIO`, `LED`, and `Servo` classes.

```python
from gpio import GPIO, LED, Servo
from time import sleep

def main() -> None:
    adc = GPIO(35, pinType="analog")
    digpin = GPIO(12, pinType="digital", mode="IN")
    led = GPIO(14, pinType="pwm")
    led2 = GPIO(4, pinType="digital", mode="OUT")
    led3 = LED(15, pinType="pwm")
    servo = Servo(16)

    print(adc.read())
    print(digpin.read())
    led.write(dutyCycle=10)

    led3.fadeIn()
    sleep(1)
    led3.fadeOut()

    servo.move(90)
    sleep(1)
    servo.move(0)

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