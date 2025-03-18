from machine import Pin, ADC, PWM, time_pulse_us
from time import sleep, sleep_ms, sleep_us

CODE = {
    'A' : '.-',
    'B' : '-...',
    'C' : '-.-.',
    'D' : '-..',
    'E' : '.',
    'F' : '..-.',
    'G' : '--.',
    'H' : '....',
    'I' : '..',
    'J' : '.---',
    'K' : '-.-',
    'L' : '.-..',
    'M' : '--',
    'N' : '-.',
    'O' : '---',
    'P' : '.--.',
    'Q' : '--.-',
    'R' : '.-.',
    'S' : '...',
    'T' : '-',
    'U' : '..-',
    'V' : '...-',
    'W' : '.--',
    'X' : '-..-',
    'Y' : '-.--',
    'Z' : '--..',
    '0' : '-----',
    '1' : '.----',
    '2' : '..---',
    '3' : '...--',
    '4' : '....-',
    '5' : '.....',
    '6' : '-....',
    '7' : '--...',
    '8' : '---..',
    '9' : '----.',
    '.' : '.-.-.-',
    ',' : '--..--',
    '?' : '..--..',
    '/' : '--..-.',
    '@' : '.--.-.',
    '!' : '-.-.--',
    ' ' : ' '
}

class GPIO:
    """
    A class to represent a GPIO pin.

    Attributes:
    -----------
    pin : machine.Pin, machine.ADC, or machine.PWM
        The pin object.
    mode : str
        The mode of the pin (e.g., "OUT", "IN", "IN_PULLUP").
    pinType : str
        The type of the pin (e.g., "digital", "analog", "pwm").
    pinNumber : int
        The pin number.
    """

    DIG = "digital"
    ADC = "analog"
    PWM = "pwm"
    OUT = "OUT"
    IN = "IN"
    IN_PULLUP = "IN_PULLUP"

    def __init__(self, pin, pinType=DIG, mode=None):
        """
        Initialize the GPIO pin.

        Parameters:
        -----------
        pin : int
            The pin number.
        pinType : str, optional
            The type of the pin (default is "digital").
        mode : str, optional
            The mode of the pin (default is None).
        """
        self.mode = mode
        self.pinType = pinType
        self.pinNumber = pin
        if pinType == "digital":
            if mode == "OUT":
                self.pin = Pin(pin, Pin.OUT)
            elif mode == "IN":
                self.pin = Pin(pin, Pin.IN)
            elif mode == "IN_PULLUP":
                self.pin = Pin(pin, Pin.IN, Pin.PULL_UP)
        elif pinType == "analog":
            self.pin = ADC(pin)
        elif pinType == "pwm":
            self.pin = PWM(pin)
    
    def init(self, mode=None):
        """
        Reinitialize the GPIO pin with a new mode.

        Parameters:
        -----------
        mode : str, optional
            The mode to set the pin to (e.g., "OUT", "IN", "IN_PULLUP").
        """
        if self.pinType == GPIO.DIG:
            if mode == "OUT":
                self.pin.init(Pin.OUT)
            elif mode == "IN":
                self.pin.init(Pin.IN)
            elif mode == "IN_PULLUP":
                self.pin.init(Pin.IN, Pin.PULL_UP)
        else:
            raise TypeError("Pin has to be set as digital")
    
    def setFreq(self, freq=1000):
        """
        Set the frequency for a PWM pin.

        Parameters:
        -----------
        freq : int, optional
            The frequency in Hz (default is 1000).

        Raises:
        -------
        TypeError
            If the pin is not a PWM pin.
        """
        if self.pinType == "pwm":
            self.pin.freq(freq)

    def flash(self, t=1):
        """
        Flash the digital output pin for a specified duration.

        Parameters:
        -----------
        t : int, optional
            The duration to flash the pin in seconds (default is 1).
        """
        if self.pinType == "digital" and self.mode == "OUT":
            print("Flashing pin number " + str(self.pinNumber) + "for " + str(t) + " Seconds")
            self.pin.on()
            sleep(t)
            self.pin.off()
            print("Done")
        else:
            raise TypeError("Pin has to be digital and set as OUT")
    
    def read(self):
        """
        Read the value from the pin.

        Returns:
        --------
        int
            The value read from the pin.
        """
        if self.pinType == "analog":
            return self.pin.read_u16()
        elif self.pinType == "digital" and self.mode == "IN":
            return self.pin.value()
        elif self.pinType == "digital" and self.mode == "IN_PULLUP":
            return self.pin.value()
        else:
            raise TypeError("Pin has to be either digital and set as IN or analog")
    
    def write(self, value=None):
        """
        Write a value to the pin.

        Parameters:
        -----------
        value : int, optional
        TypeError
            If the pin is not digital and set as OUT or PWM.
        """
        if self.pinType == "digital" and self.mode == "OUT":
            if value == 0 or value == 1:
                self.pin.value(value)
            else:
                raise ValueError("State has to either be 1 or 0")
        elif self.pinType == "pwm":
            if value > 1023 or value < 0:
                raise ValueError("Duty cycle must be between 0 and 1023")
            else:
                self.pin.duty(value)
        else:
            raise TypeError("Pin has to be either digital and set as OUT or PWM")
    
    def toggle(self):
        """
        Toggle the digital output pin.

        Raises:
        -------
        TypeError
            If the pin is not digital and set as OUT.
        """
        if self.pinType == "digital" and self.mode == "OUT":
            if self.pin.value() == 0:
                self.pin.value(1)
            else:
                self.pin.value(0)
        else:
            raise TypeError("Pin has to be digital and set as OUT")

    def attachInterrupt(self, trigger, callback):
        """
        Attach an interrupt to the GPIO pin.

        Parameters:
        -----------
        trigger : int
            The type of trigger for the interrupt (e.g., Pin.IRQ_RISING, Pin.IRQ_FALLING, or Pin.IRQ_RISING | Pin.IRQ_FALLING).
        callback : function
            The callback function to execute when the interrupt is triggered.

        Raises:
        -------
        TypeError
            If the pin is not digital or set as IN/IN_PULLUP.
        """
        if self.pinType == GPIO.DIG and (self.mode == GPIO.IN or self.mode == GPIO.IN_PULLUP):
            self.pin.irq(trigger=trigger, handler=callback)
        else:
            raise TypeError("Interrupts can only be attached to digital pins set as IN or IN_PULLUP.")


class LED:
    """
    A class to represent an LED.

    Attributes:
    -----------
    led : GPIO
        The GPIO object representing the LED.
    wpm : int
        Words per minute for Morse code.
    """

    def __init__(self, pin, pinType=GPIO.DIG, freq=1000):
        """
        Initialize the LED.

        Parameters:
        -----------
        pin : int
            The pin number.
        pinType : str, optional
            The type of the pin (default is "digital").
        freq : int, optional
            The PWM frequency (default is 1000).
        """
        self.wpm = 15
        if pinType == "digital":
            self.led = GPIO(pin, GPIO.DIG, mode="OUT")
        elif pinType == "pwm":
            self.led = GPIO(pin, GPIO.PWM)
            self.led.setFreq(freq)
    
    def flash(self, t=1):
        """
        Flash the LED.
        Default time is 1 second.

        Parameters:
        -----------
        t : int, optional
            The duration to flash the pin in seconds (default is 1).

        Raises:
        -------
        TypeError
            If the pin is not digital or PWM.
        """
        if self.led.pinType == "digital":
            self.led.write(1)
            sleep(t)
            self.led.write(0)
        elif self.led.pinType == "pwm":
            self.led.write(1023)
            sleep(t)
            self.led.write(0)
        else:
            raise TypeError("Pin has to be digital or PWM")
    
    def blink(self, n, t=0.15):
        """
        Blink the LED a specified number of times.

        Parameters:
        -----------
        n : int
            The number of times to blink the LED.
        t : float, optional
            The duration between blinks in seconds (default is 0.15).
        """
        for i in range(n):
            self.flash(0.05)
            sleep(t)
    
    def fadeIn(self):
        """
        Gradually increase the brightness of the LED.

        Raises:
        -------
        TypeError
            If the pin is not PWM.
        """
        if self.led.pinType == "pwm":
            for i in range(0, 1023, 3):
                self.led.write(i)
                sleep(0.02)
            self.led.write(1023)
            
        else:
            raise TypeError("Pin has to be PWM")

    def fadeOut(self):
        """
        Gradually decrease the brightness of the LED.

        Raises:
        -------
        TypeError
            If the pin is not PWM.
        """
        if self.led.pinType == "pwm":
            for i in range(1023, 0, -3):
                self.led.write(i)
                sleep(0.02)
            self.led.write(0)
        else:
            raise TypeError("Pin has to be PWM")
    
    def morsecode(self, msg="SOS"):
        """
        Flash the LED in Morse code.

        Parameters:
        -----------
        msg : str, optional
            The message to flash in Morse code (default is "SOS").

        Raises:
        -------
        TypeError
            If the pin is not digital or PWM.
        """
        tdot = 1.2/self.wpm
        tdash = tdot * 3.5
        tspace = tdot * 2
        tword = tdot * 6
        if self.led.pinType == "digital":
            self.led.write(0)
        elif self.led.pinType == "pwm":
            self.led.write(0)
        else:
            raise TypeError("Pin has to be digital or PWM")
        for l in msg:
            c = CODE.get(l.upper())
            for e in c:
                if e==".":
                    self.flash(tdot)
                    sleep(tdot)
                if e=="-":
                    self.flash(tdash)
                    sleep(tdot)
                if e==" ": sleep(tword)
                sleep(tword)
        if self.led.pinType == "digital":
            self.led.write(0)
        elif self.led.pinType == "pwm":
            self.led.write(0)

    def setMorseSpeed(self, speed):
        """
        Set the speed of the Morse code.
        
        Parameters:
        -----------
        speed : int
            The speed of the Morse code in words per minute.
        """
        self.wpm = speed


class Servo:
    """
    A class to represent a Servo motor.

    Attributes:
    -----------
    pwm : GPIO
        The GPIO object representing the PWM pin.
    mode : str
        The mode of the servo (e.g., "absolute", "relative").
    minUs : int
        Minimum pulse width in microseconds.
    maxUs : int
        Maximum pulse width in microseconds.
    maxAngle : int
        Maximum angle of the servo.
    angle : int
        Current angle of the servo.
    speed : int
        Speed of the servo movement.
    """

    def __init__(self, pin, mode="absolute", freq=50, minUs=500, maxUs=2500, maxAngle=180):
        """
        Initialize the Servo motor.

        Parameters:
        -----------
        pin : int
            The pin number.
        mode : str, optional
            The mode of the servo (default is "absolute").
        freq : int, optional
            The PWM frequency (default is 50).
        minUs : int, optional
            Minimum pulse width in microseconds (default is 500).
        maxUs : int, optional
            Maximum pulse width in microseconds (default is 2500).
        maxAngle : int, optional
            Maximum angle of the servo (default is 180).
        """
        self.pwm = GPIO(pin, GPIO.PWM)
        self.pwm.setFreq(freq)
        self.mode = mode
        self.minUs = minUs
        self.maxUs = maxUs
        self.maxAngle = maxAngle
        self.angle = 90  # Current angle
        self.speed = 1
        self.reset()

    def _angleToDuty(self, angle):
        """
        Convert an angle to a duty cycle.

        Parameters:
        -----------
        angle : int
            The angle to convert.

        Returns:
        --------
        int
            The duty cycle corresponding to the angle.
        """
        return int(((angle / self.maxAngle) * (self.maxUs - self.minUs) + self.minUs) / 20000 * 1023)

    def move(self, targetAngle):
        """
        Move the servo to a target angle.

        Parameters:
        -----------
        targetAngle : int
            The target angle to move the servo to.
        """
        if self.mode == "relative":
            targetAngle += self.angle

        elif self.mode == "absolute":
            targetAngle = max(0, min(self.maxAngle, targetAngle))  # Constrain to valid range
            step = 1 if targetAngle > self.angle else -1

            for angle in range(self.angle, targetAngle + step, step):
                self.pwm.write(self._angleToDuty(angle))
                self.angle = angle
                sleep(self.speed / 1000)  # Speed in milliseconds

    def reset(self):
        """
        Reset the servo to the 0 degree position.
        """
        tmp = self.mode
        self.mode = "absolute"
        self.move(0)
        self.mode = tmp

    def mid(self):
        """
        Move the servo to the middle position.
        """
        tmp = self.mode
        self.mode = "absolute"
        self.move(self.maxAngle // 2)
        self.mode = tmp

    def max(self):
        """
        Move the servo to the max position.
        """
        tmp = self.mode
        self.mode = "absolute"
        self.move(self.maxAngle)
        self.mode = tmp

class Stepper:
    """
    A class to represent a stepper motor.

    Attributes:
    -----------
    stp : GPIO
        The GPIO object for the step pin.
    dir : GPIO
        The GPIO object for the direction pin.
    slp : GPIO
        The GPIO object for the sleep pin.
    step_time : int
        The time between steps in microseconds.
    steps_per_rev : int
        The number of steps per revolution.
    current_position : int
        The current position of the motor in steps.
    """
    def __init__(self, step_pin, dir_pin, sleep_pin):
        self.stp = GPIO(step_pin, GPIO.DIG, GPIO.OUT)
        self.dir = GPIO(dir_pin, GPIO.DIG, GPIO.OUT)
        self.slp = GPIO(sleep_pin, GPIO.DIG, GPIO.OUT)

        self.step_time = 20  # us
        self.steps_per_rev = 200
        self.current_position = 0

    def powerOn(self):
        self.slp.write(1)

    def powerOff(self):
        self.slp.write(0)
        self.current_position = 0

    def steps(self, step_count):
        self.dir.write(1 if step_count > 0 else 0)
        for i in range(abs(step_count)):
            self.stp.write(1)
            sleep_us(self.step_time)
            self.stp.write(0)
            sleep_us(self.step_time)
        self.current_position += step_count

    def relAngle(self, angle):
        steps = int(angle / 360 * self.steps_per_rev)
        self.steps(steps)

    def absAngle(self, angle):
        steps = int(angle / 360 * self.steps_per_rev)
        steps -= self.current_position % self.steps_per_rev
        self.steps(steps)

    def revolution(self, rev_count):
        self.steps(rev_count * self.steps_per_rev)

    def setStepTime(self, us):
        if us < 20:
            self.step_time = 20
        else:
            self.step_time = us

class StepperULN:
    """
    A class to represent a stepper motor controlled by a ULN2003 driver.

    Attributes:
    -----------
    pin1, pin2, pin3, pin4 : GPIO
        GPIO objects for the motor control pins.
    delay : int
        Delay between steps in milliseconds.
    mode : list
        Step sequence for the motor (full-step or half-step).
    """
    FULL_ROTATION = int(4075.7728395061727 / 8) # http://www.jangeox.be/2013/10/stepper-motor-28byj-48_25.html

    HALF_STEP = [
        [0, 0, 0, 1],
        [0, 0, 1, 1],
        [0, 0, 1, 0],
        [0, 1, 1, 0],
        [0, 1, 0, 0],
        [1, 1, 0, 0],
        [1, 0, 0, 0],
        [1, 0, 0, 1],
    ]

    FULL_STEP = [
        [1, 0, 1, 0],
        [0, 1, 1, 0],
        [0, 1, 0, 1],
        [1, 0, 0, 1]
    ]

    FULLSTEP = "fs"
    HALFSTEP = "hs"

    def __init__(self, pin1, pin2, pin3, pin4, delay, mode=FULLSTEP):
        """
        Initialize the stepper motor.

        Parameters:
        -----------
        pin1, pin2, pin3, pin4 : int
            Pin numbers for the motor control.
        delay : int
            Delay between steps in milliseconds.
        mode : str, optional
            Step mode (FULLSTEP or HALFSTEP, default is FULLSTEP).
        """
        if mode == "fs":
            self.mode = self.FULL_STEP
        elif mode == "hs":
            self.mode = self.HALF_STEP
        else:
            raise ValueError("Mode must be either 0 or 1")
        self.pin1 = GPIO(pin1, GPIO.DIG, GPIO.OUT)
        self.pin2 = GPIO(pin2, GPIO.DIG, GPIO.OUT)
        self.pin3 = GPIO(pin3, GPIO.DIG, GPIO.OUT)
        self.pin4 = GPIO(pin4, GPIO.DIG, GPIO.OUT)
        self.delay = delay
        
        self.reset()
        
    def step(self, count, direction=1):
        """
        Move the motor by a specified number of steps.

        Parameters:
        -----------
        count : int
            Number of steps to move.
        direction : int, optional
            Direction of movement (1 for forward, -1 for backward, default is 1).
        """
        if count<0:
            direction = -1
            count = -count
        for x in range(count):
            for bit in self.mode[::direction]:
                self.pin1.write(bit[0])
                self.pin2.write(bit[1])
                self.pin3.write(bit[2])
                self.pin4.write(bit[3])
                sleep_ms(self.delay)
        self.reset()

    def angle(self, r, direction=1):
        """
        Rotate the motor by a specified angle.

        Parameters:
        -----------
        r : float
            Angle in degrees.
        direction : int, optional
            Direction of rotation (1 for forward, -1 for backward, default is 1).
        """
        self.step(int(self.FULL_ROTATION * r / 360), direction)

    def reset(self):
        """
        Reset the motor pins to low state.
        """
        self.pin1(0) 
        self.pin2(0) 
        self.pin3(0) 
        self.pin4(0)

class UltraSonic:
    """
    A class to represent an ultrasonic distance sensor.

    Attributes:
    -----------
    trigger : GPIO
        GPIO object for the trigger pin.
    echo : GPIO
        GPIO object for the echo pin.
    echo_timeout_us : int
        Timeout for the echo signal in microseconds.
    """
    def __init__(self, trigger_pin=12, echo_pin=14):
        self.echo_timeout_us = 500*2*30
        self.trigger = GPIO(trigger_pin, GPIO.DIG, GPIO.OUT)
        self.trigger.write(0)
        self.echo = GPIO(echo_pin, GPIO.DIG, GPIO.IN)

    def _sendPulseAndWait(self):
        # Send the pulse to trigger and listen on echo pin.
        self.trigger.write(0) # Stabilize the sensor
        sleep_us(5)
        self.trigger.write(1)
        # Send a 10us pulse.
        sleep_us(10)
        self.trigger.write(0)
        try:
            pulse_time = time_pulse_us(self.echo, 1, self.echo_timeout_us)
            return pulse_time
        except OSError as ex:
            if ex.args[0] == 110:
                raise OSError('Out of range')
            raise ex

    def getDistanceMm(self):
        pulse_time = self._send_pulse_and_wait() 
        mm = pulse_time * 100 // 582
        return mm

    def getDistanceCm(self):
        return (self.get_distance_mm()/10)


class Joystick:
    """
    A class to represent a joystick.

    Attributes:
    -----------
    jsx : GPIO
        GPIO object for the X-axis.
    jsy : GPIO
        GPIO object for the Y-axis.
    jsb : GPIO
        GPIO object for the button.
    deadzone : int
        Deadzone threshold for the joystick.
    sensitivityX : float
        Sensitivity for the X-axis.
    sensitivityY : float
        Sensitivity for the Y-axis.
    centerX : int
        Calibrated center position for the X-axis.
    centerY : int
        Calibrated center position for the Y-axis.
    moveCallback : function
        Callback function for joystick movement.
    buttonCallback : function
        Callback function for button presses.
    lastX : int
        Last X-axis value to detect changes.
    lastY : int
        Last Y-axis value to detect changes.
    lastBtn : int
        Last button state to detect changes.
    mapRange : tuple
        Range to map joystick readings to (default is None).
    """

    def __init__(self, x, y, btn, deadzone=100, mapRange=None):
        """
        Initialize the joystick.

        Parameters:
        -----------
        x : int
            Pin number for the X-axis.
        y : int
            Pin number for the Y-axis.
        btn : int
            Pin number for the button.
        deadzone : int, optional
            Deadzone threshold for the joystick (default is 100).
        mapRange : tuple, optional
            Range to map joystick readings to (e.g., (-1, 1), default is None).
        """
        self.jsx = GPIO(x, GPIO.ADC)
        self.jsy = GPIO(y, GPIO.ADC)
        self.jsb = GPIO(btn, GPIO.DIG, GPIO.IN_PULLUP)
        self.deadzone = deadzone
        self.sensitivityX = 1.0
        self.sensitivityY = 1.0
        self.centerX = 32768  # Assuming 16-bit ADC
        self.centerY = 32768
        self.moveCallback = None
        self.buttonCallback = None
        self.lastX = None
        self.lastY = None
        self.lastBtn = None
        self.mapRange = mapRange

    def mapValue(self, value, inMin, inMax, outMin, outMax):
        """
        Map a value from one range to another.

        Parameters:
        -----------
        value : int
            The value to map.
        inMin : int
            Minimum of the input range.
        inMax : int
            Maximum of the input range.
        outMin : int
            Minimum of the output range.
        outMax : int
            Maximum of the output range.

        Returns:
        --------
        int
            The mapped value.
        """
        return (value - inMin) * (outMax - outMin) // (inMax - inMin) + outMin

    def read(self):
        """
        Read the joystick values.

        Returns:
        --------
        tuple
            The X and Y values and the button state.
        """
        x = self.jsx.read()
        y = self.jsy.read()
        btn = self.jsb.read()

        if self.mapRange:
            x = self.mapValue(x, 0, 65535, self.mapRange[0], self.mapRange[1])
            y = self.mapValue(y, 0, 65535, self.mapRange[0], self.mapRange[1])

        self.poll(x, y, btn)
        return (x, y, btn)

    def applyDeadzone(self, value):
        """
        Apply the deadzone to a joystick axis value.

        Parameters:
        -----------
        value : int
            The raw value from the joystick axis.

        Returns:
        --------
        int
            The adjusted value after applying the deadzone.
        """
        if abs(value - 32768) < self.deadzone:
            return 32768
        return value

    def isButtonPressed(self):
        """
        Check if the joystick button is pressed with debouncing.

        Returns:
        --------
        bool
            True if the button is pressed, False otherwise.
        """
        state = self.jsb.read()
        sleep_ms(10)  # Debounce delay
        return state == 0 and self.jsb.read() == 0

    def getDirection(self):
        """
        Get the direction of the joystick.

        Returns:
        --------
        str
            The direction of the joystick ("UP", "DOWN", "LEFT", "RIGHT", "CENTER").
        """
        x = self.applyDeadzone(self.jsx.read())
        y = self.applyDeadzone(self.jsy.read())

        if y > 40000: return "UP"
        if y < 25000: return "DOWN"
        if x > 40000: return "RIGHT"
        if x < 25000: return "LEFT"
        return "CENTER"

    def calibrate(self):
        """
        Calibrate the joystick by setting the center position.
        """
        self.centerX = self.jsx.read()
        self.centerY = self.jsy.read()

    def readCalibrated(self):
        """
        Read the joystick values relative to the calibrated center.

        Returns:
        --------
        tuple
            The calibrated X and Y values.
        """
        x = self.jsx.read() - self.centerX
        y = self.jsy.read() - self.centerY
        return (x, y)

    def onMove(self, callback):
        """
        Set a callback function for joystick movement.

        Parameters:
        -----------
        callback : function
            The function to call when the joystick moves.
        """
        self.moveCallback = callback

    def onButtonPress(self, callback):
        """
        Set a callback function for button presses.

        Parameters:
        -----------
        callback : function
            The function to call when the button is pressed.
        """
        self.buttonCallback = callback

    def poll(self, x, y, btn):
        """
        Poll the joystick for changes and trigger callbacks.

        Parameters:
        -----------
        x : int
            Current X-axis value.
        y : int
            Current Y-axis value.
        btn : int
            Current button state.
        """
        if (x != self.lastX or y != self.lastY) and self.moveCallback:
            self.moveCallback(x, y)
        if btn != self.lastBtn and self.buttonCallback and btn == 1:
            self.buttonCallback()

        self.lastX = x
        self.lastY = y
        self.lastBtn = btn

    def setSensitivity(self, sensitivityX=1.0, sensitivityY=1.0):
        """
        Set the sensitivity for the joystick axes.

        Parameters:
        -----------
        sensitivityX : float, optional
            Sensitivity for the X-axis (default is 1.0).
        sensitivityY : float, optional
            Sensitivity for the Y-axis (default is 1.0).
        """
        self.sensitivityX = sensitivityX
        self.sensitivityY = sensitivityY

    def readWithSensitivity(self):
        """
        Read the joystick values with sensitivity applied.

        Returns:
        --------
        tuple
            The adjusted X and Y values.
        """
        x = self.jsx.read() * self.sensitivityX
        y = self.jsy.read() * self.sensitivityY
        return (x, y)