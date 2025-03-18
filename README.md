# MicroPython GPIO Driver

This project provides a simple GPIO driver for MicroPython, allowing you to easily control digital, analog, PWM pins, and various peripherals on your microcontroller.

## Files

- `boot.py`: Entry point for the MicroPython script.
- `gpio.py`: Contains the `GPIO`, `LED`, `Servo`, `Stepper`, `StepperULN`, `UltraSonic`, and `Joystick` classes.
- `main.py`: Example usage of the classes.

## Usage

### GPIO Class

The `GPIO` class allows you to initialize and control GPIO pins. It supports digital, analog, and PWM pins.

#### Initialization

```python
from gpio import GPIO

# Digital pin as output
led = GPIO(14, GPIO.DIG, GPIO.OUT)

# Analog pin
adc = GPIO(35, GPIO.ADC)

# PWM pin
pwm_led = GPIO(25, GPIO.PWM)
```

#### Methods

- `flash(t=1)`: Flash the digital output pin for `t` seconds.
- `read()`: Read the value from the pin.
- `write(value=None)`: Write a value to the pin.
- `toggle()`: Toggle the digital output pin.
- `attach_interrupt(trigger, callback)`: Attach an interrupt to the GPIO pin.

### LED Class

The `LED` class allows you to control an LED with digital or PWM pins.

#### Initialization

```python
from gpio import LED

# Digital LED
led = LED(14, GPIO.DIG)

# PWM LED
pwm_led = LED(25, GPIO.PWM)
```

#### Methods

- `flash(t=1)`: Flash the LED for `t` seconds.
- `blink(n, t=0.15)`: Blink the LED `n` times with `t` seconds between blinks.
- `fadeIn()`: Gradually increase the brightness of the LED.
- `fadeOut()`: Gradually decrease the brightness of the LED.
- `morsecode(msg="SOS")`: Flash the LED in Morse code for the given message.
- `setMorseSpeed(speed)`: Set the speed of the Morse code in words per minute.

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

### Stepper Class

The `Stepper` class allows you to control a stepper motor.

#### Initialization

```python
from gpio import Stepper

stepper = Stepper(step_pin=17, dir_pin=18, sleep_pin=19)
```

#### Methods

- `power_on()`: Enable the stepper motor.
- `power_off()`: Disable the stepper motor.
- `steps(step_count)`: Move the motor by a specified number of steps.
- `rel_angle(angle)`: Rotate the motor by a relative angle.
- `abs_angle(angle)`: Rotate the motor to an absolute angle.
- `revolution(rev_count)`: Rotate the motor by a specified number of revolutions.
- `set_step_time(us)`: Set the time between steps in microseconds.

### StepperULN Class

The `StepperULN` class allows you to control a stepper motor using a ULN2003 driver.

#### Initialization

```python
from gpio import StepperULN

stepper_uln = StepperULN(pin1=25, pin2=26, pin3=27, pin4=28, delay=5, mode=StepperULN.HALFSTEP)
```

#### Methods

- `step(count, direction=1)`: Move the motor by a specified number of steps.
- `angle(r, direction=1)`: Rotate the motor by a specified angle.
- `reset()`: Reset the motor pins to low state.

### UltraSonic Class

The `UltraSonic` class allows you to measure distances using an ultrasonic sensor.

#### Initialization

```python
from gpio import UltraSonic

ultrasonic = UltraSonic(trigger_pin=20, echo_pin=21)
```

#### Methods

- `get_distance_mm()`: Get the distance in millimeters.
- `get_distance_cm()`: Get the distance in centimeters.

### Joystick Class

The `Joystick` class allows you to read values from a joystick.

#### Initialization

```python
from gpio import Joystick

joystick = Joystick(x=22, y=23, btn=24)
```

#### Methods

- `read()`: Read the joystick's X, Y, and button states. Returns a tuple `(x, y, btn)` where:
  - `x`: ADC value of the X-axis.
  - `y`: ADC value of the Y-axis.
  - `btn`: Button state (1 for pressed, 0 for not pressed).