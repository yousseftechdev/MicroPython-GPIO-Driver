# MicroPython GPIO Driver

This project provides a simple GPIO driver for MicroPython, allowing you to easily control digital, analog, PWM pins, and various peripherals on your microcontroller.

## Files

- `boot.py`: Entry point for the MicroPython script.
- `gpio.py`: Contains the `GPIO`, `LED`, `Servo`, `Stepper`, `StepperULN`, `UltraSonic`, `Joystick`, `RotaryEncoder`, `SevenSegment`, and `RGB` classes.
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

- `init(mode=None)`: Reinitialize the GPIO pin with a new mode.
- `setFreq(freq=1000)`: Set the frequency for a PWM pin.
- `flash(t=1)`: Flash the digital output pin for `t` seconds.
- `read()`: Read the value from the pin.
- `write(value=None)`: Write a value to the pin.
- `toggle()`: Toggle the digital output pin.
- `attachInterrupt(trigger, callback)`: Attach an interrupt to the GPIO pin.

---

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

- `on()`: Turn the LED on.
- `off()`: Turn the LED off.
- `flash(t=1)`: Flash the LED for `t` seconds.
- `blink(n, t=0.15)`: Blink the LED `n` times with `t` seconds between blinks.
- `fadeIn()`: Gradually increase the brightness of the LED.
- `fadeOut()`: Gradually decrease the brightness of the LED.
- `write(value=None)`: Write a PWM duty cycle to the LED.
- `morseCode(msg="SOS")`: Flash the LED in Morse code for the given message.
- `setMorseSpeed(speed)`: Set the speed of the Morse code in words per minute.

---

### RGB Class

The `RGB` class allows you to control an RGB LED with digital or PWM pins.

#### Initialization

```python
from gpio import RGB

# RGB LED with PWM pins
rgb = RGB(r=14, g=15, b=16, pinType=RGB.PWM, mode=RGB.COMM_CATHODE)
```

#### Methods

- `flash(led, t=1)`: Flash a specific LED (`RGB.RED`, `RGB.GREEN`, or `RGB.BLUE`) for `t` seconds.
- `writeDig(r_val, g_val, b_val)`: Write digital values to the RGB LED.
- `write(r_val, g_val, b_val)`: Write PWM values to the RGB LED.
- `blink(r_val, g_val, b_val, t=1, n=1)`: Blink the RGB LED with specified values for `n` times.

---

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
- `setMode(mode)`: Sets the movement mode of the servo. Takes either `Servo.ABS` or `Servo.REL` as args.
- `reset()`: Reset the servo to the 0-degree position.
- `mid()`: Move the servo to the middle position.
- `max()`: Move the servo to the max degree position.

---

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

---

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
- `reset()`: Reset the motor pins to a low state.

---

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

- `read()`: Read the joystick's X, Y, and button states. Returns a tuple `(x, y, btn)`.
- `getDirection()`: Get the direction of the joystick. Returns `UP/DOWN/LEFT/RIGHT/MIDDLE`.

---

### RotaryEncoder Class

The `RotaryEncoder` class allows you to handle rotary encoder inputs.

#### Initialization

```python
from gpio import RotaryEncoder

rotary_encoder = RotaryEncoder(clkPin=29, dtPin=30, swPin=31)
```

#### Methods

- `onRight(handler)`: Register a callback function for right rotation.
- `onLeft(handler)`: Register a callback function for left rotation.
- `onButtonPress(handler)`: Register a callback function for button press.

---

### SevenSegment Class

The `SevenSegment` class allows you to control a 7-segment display.

#### Initialization

```python
from gpio import SevenSegment

seven_seg = SevenSegment(segment_pins=[2, 3, 4, 5, 6, 7, 8], common_anode=True)
```

#### Methods

- `display(digit)`: Display a digit (0-9) on the 7-segment display.
- `clear()`: Clear the 7-segment display.
- `random()`: Display a random digit on the 7-segment display.
- `countup(count=10, t=1)`: Count up on the 7-segment display.
- `countdown(count=10, t=1)`: Count down on the 7-segment display.