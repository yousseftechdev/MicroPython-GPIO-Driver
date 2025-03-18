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
pwmLed = GPIO(25, GPIO.PWM)
```

#### Methods

- `flash(t=1)`: Flash the digital output pin for `t` seconds.
- `read()`: Read the value from the pin.
- `write(value=None)`: Write a value to the pin.
- `toggle()`: Toggle the digital output pin.
- `attachInterrupt(trigger, callback)`: Attach an interrupt to the GPIO pin.

### LED Class

The `LED` class allows you to control an LED with digital or PWM pins.

#### Initialization

```python
from gpio import LED

# Digital LED
led = LED(14, GPIO.DIG)

# PWM LED
pwmLed = LED(25, GPIO.PWM)
```

#### Methods

- `flash(t=1)`: Flash the LED for `t` seconds.
- `blink(n, t=0.15)`: Blink the LED `n` times with `t` seconds between blinks.
- `fadeIn()`: Gradually increase the brightness of the LED.
- `fadeOut()`: Gradually decrease the brightness of the LED.
- `morseCode(msg="SOS")`: Flash the LED in Morse code for the given message.
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

stepper = Stepper(stepPin=17, dirPin=18, sleepPin=19)
```

#### Methods

- `powerOn()`: Enable the stepper motor.
- `powerOff()`: Disable the stepper motor.
- `steps(stepCount)`: Move the motor by a specified number of steps.
- `relAngle(angle)`: Rotate the motor by a relative angle.
- `absAngle(angle)`: Rotate the motor to an absolute angle.
- `revolution(revCount)`: Rotate the motor by a specified number of revolutions.
- `setStepTime(us)`: Set the time between steps in microseconds.

### StepperULN Class

The `StepperULN` class allows you to control a stepper motor using a ULN2003 driver.

#### Initialization

```python
from gpio import StepperULN

stepperULN = StepperULN(pin1=25, pin2=26, pin3=27, pin4=28, delay=5, mode=StepperULN.HALFSTEP)
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

ultraSonic = UltraSonic(triggerPin=20, echoPin=21)
```

#### Methods

- `getDistanceMm()`: Get the distance in millimeters.
- `getDistanceCm()`: Get the distance in centimeters.

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
- `calibrate()`: Calibrate the joystick to set the center position.
- `isPressed()`: Check if the joystick button is pressed. Returns `True` if pressed, otherwise `False`.
- `setSensitivity(level)`: Set the sensitivity level of the joystick. Higher levels make the joystick more responsive.
- `getDirection()`: Get the direction of the joystick movement. Returns one of `["UP", "DOWN", "LEFT", "RIGHT", "CENTER"]`.
- `onMove(callback)`: Register a callback function that is triggered when the joystick is moved. The callback receives the x and y values as an arguments.
- `onButtonPress(callback)`: Register a callback function that is triggered when the joystick button is pressed.