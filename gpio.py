from machine import Pin, ADC, PWM, time_pulse_us
from utime import sleep, sleep_ms, sleep_us
from urandom import randint


class GPIO:
    """
    A class to represent a GPIO pin.

    Attributes:
    -----------
    pin : machine.Pin, machine.ADC, or machine.PWM
        The pin object.
    mode : str
        The mode of the pin (e.g., "OUT", "IN", "IN_PULLUP", "IN_PULLDOWN").
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
    IN_PULLDOWN = "IN_PULLDOWN"
    RISING = 0x0001
    FALLING = 0x0002

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
            elif mode == "IN_PULLDOWN":
                self.pin = Pin(pin, Pin.IN, Pin.PULL_DOWN)
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
            elif mode == "IN_PULLDOWN":
                self.pin.init(Pin.IN, Pin.PULL_DOWN)
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
        elif self.pinType == "digital" and (self.mode == "IN" or self.mode == "IN_PULLUP" or self.mode == "IN_PULLDOWN"):
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
            The type of trigger for the interrupt (e.g., GPIO.RISING, GPIO.FALLING, or GPIO.RISING | GPIO.FALLING).
        callback : function
            The callback function to execute when the interrupt is triggered.

        Raises:
        -------
        TypeError
            If the pin is not digital or set as IN/IN_PULLUP/IN_PULLDOWN.
        """
        if self.pinType == self.DIG and (self.mode == self.IN or self.mode == self.IN_PULLUP or self.mode == self.IN_PULLDOWN):
            if trigger in (self.RISING, self.FALLING, self.RISING | self.FALLING):
                self.pin.irq(trigger=trigger, handler=callback)
            else:
                raise ValueError("Provided trigger was invalid.")
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
    DIG = "digital"
    PWM = "pwm"

    def __init__(self, pin, pinType=DIG, freq=1000):
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
        self.pinType = pinType
        self.wpm = 15
        if pinType == "digital":
            self.led = GPIO(pin, GPIO.DIG, mode="OUT")
        elif pinType == "pwm":
            self.led = GPIO(pin, GPIO.PWM)
            self.led.setFreq(freq)
            
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
        if self.led.pinType == "pwm":
            self.led.setFreq(freq)
        else:
            raise TypeError("Pin has to be PWM")
    
    def on(self):
        if self.led.pinType == "digital":
            self.led.write(1)
        elif self.led.pinType == "pwm":
            self.led.write(1023)
        else:
            raise TypeError("Pin has to be digital or PWM")

    def off(self):
        if self.led.pinType in ("digital", "pwm"):
            self.led.write(0)
        else:
            raise TypeError("Pin has to be digital or PWM")
    
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
        if self.led.pinType in ("digital", "pwm"):
            self.on()
            sleep(t)
            self.off()
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
    
    def write(self, value=None):
        if self.pinType == "pwm":
            if value > 1023 or value < 0:
                raise ValueError("Duty cycle must be between 0 and 1023")
            else:
                self.led.write(value)
        else:
            raise TypeError("Pin has to be PWM")
    
    def morsecode(self, msg="SOS"):
        """
        Flash the LED in Morse code.

        Parameters:
        -----------
        msg : str, optional
            The message to flash in Morse code (default is "SOS").
        """
        tdot = 1.2/self.wpm
        tdash = tdot * 3.5
        tspace = tdot * 2
        tword = tdot * 6
        if self.led.pinType == "digital":
            self.off()
        elif self.led.pinType == "pwm":
            self.off()
        else:
            raise TypeError("Pin has to be digital or PWM")
        for l in msg:
            c = self.CODE.get(l.upper())
            for e in c:
                if e==".":
                    self.flash(tdot)
                    sleep(tdot)
                if e=="-":
                    self.flash(tdash)
                    sleep(tdot)
                if e==" ": sleep(tword)
                sleep(tword)
        if self.led.pinType in ("digital", "pwm"):
            self.off()

    def setMorseSpeed(self, speed):
        """
        Set the speed of the Morse code.
        
        Parameters:
        -----------
        speed : int
            The speed of the Morse code in words per minute.
        """
        self.wpm = speed


class RGB:
    DIG = "digital"
    PWM = "pwm"
    COMM_ANODE = "ca"
    COMM_CATHODE = "cc"

    def __init__(self, r, g, b, pinType=DIG, mode=COMM_CATHODE, freq=1000):
        self.RED = LED(r, pinType)
        self.GREEN = LED(g, pinType)
        self.BLUE = LED(b, pinType)
        if pinType == self.PWM:
            self.RED.setFreq(freq)
            self.GREEN.setFreq(freq)
            self.BLUE.setFreq(freq)
        self.pinType = pinType
        self.mode = mode
    
    def flash(self, led, t=1):
        if led in (self.RED, self.GREEN, self.BLUE):
            if led == self.RED:
                self.RED.on()
                sleep(t)
                self.RED.off()
            elif led == self.GREEN:
                self.GREEN.on()
                sleep(t)
                self.GREEN.off()
            elif led == self.BLUE:
                self.BLUE.on()
                sleep(t)
                self.BLUE.off()
        else:
            raise TypeError("Provide RGB.RED / RGB.GREEN / RGB.BLUE")

    def writeDig(self, r_val, g_val, b_val):
        if self.pinType == "digital":

            if self.mode == self.COMM_CATHODE:
                if r_val == 1: self.RED.on()
                elif r_val == 0: self.RED.off()
                if g_val == 1: self.GREEN.on()
                elif g_val == 0: self.GREEN.off()
                if b_val == 1: self.BLUE.on()
                elif b_val == 0: self.BLUE.off()
            elif self.mode == self.COMM_ANODE:
                if r_val == 1: self.RED.off()
                elif r_val == 0: self.RED.on()
                if g_val == 1: self.GREEN.off()
                elif g_val == 0: self.GREEN.on()
                if b_val == 1: self.BLUE.off()
                elif b_val == 0: self.BLUE.on()
            else:
                raise ValueError("Invalid mode. Use RGBLED.COMM_CATHODE or RGBLED.COMM_ANODE.")
        else:
            raise TypeError("Pin type should be pwm")

    def write(self, r_val, g_val, b_val):
        if self.pinType == "pwm":
            scale = lambda x: int(x * 1023 / 255)
            r_pwm, g_pwm, b_pwm = map(scale, (r_val, g_val, b_val))
            if self.mode == self.COMM_CATHODE:
                self.RED.write(r_pwm)
                self.GREEN.write(g_pwm)
                self.BLUE.write(b_pwm)
            elif self.mode == self.COMM_ANODE:
                self.RED.write(1023 - r_pwm)
                self.GREEN.write(1023 - g_pwm)
                self.BLUE.write(1023 - b_pwm)
            else:
                raise ValueError("Invalid mode. Use RGBLED.COMM_CATHODE or RGBLED.COMM_ANODE.")
        else:
            raise TypeError("Pin type should be pwm")

    def blink(self, r_val, g_val, b_val, t=1, n=1):
        for _ in range(n):
            self.write(r_val, g_val, b_val)
            sleep(t)
            if self.mode == self.COMM_CATHODE:
                self.RED.off()
                self.GREEN.off()
                self.BLUE.off()
            elif self.mode == self.COMM_ANODE:
                self.RED.on()
                self.GREEN.on()
                self.BLUE.on()
            sleep(t)


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
    
    ABS = "absolute"
    REL = "relative"

    def __init__(self, pin, mode=ABS, freq=50, minUs=500, maxUs=2500, maxAngle=180):
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
        self.REDeset()

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
    
    def setMode(self, mode):
        if mode in (self.ABS, self.REL):
            self.mode = mode
            

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
        
        self.REDeset()
        
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
        self.REDeset()

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
        """
        Initializes the GPIO driver for a trigger and echo pin setup.
        Args:
            trigger_pin (int, optional): The GPIO pin number used for the trigger signal. Defaults to 12.
            echo_pin (int, optional): The GPIO pin number used for the echo signal. Defaults to 14.
        Attributes:
            echo_timeout_us (int): Timeout value in microseconds for the echo signal.
            trigger (GPIO): GPIO object configured for the trigger pin in output mode.
            echo (GPIO): GPIO object configured for the echo pin in input mode.
        """
        self.echo_timeout_us = 500*2*30
        self.trigger = GPIO(trigger_pin, GPIO.DIG, GPIO.OUT)
        self.trigger.write(0)
        self.echo = GPIO(echo_pin, GPIO.DIG, GPIO.IN)

    def _send_pulse_and_wait(self):
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
        return (self.GREENetDistanceMm()/10)


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
    """
    def __init__(self, x, y, btn):
        """
        Initialize the joystick.
        
        """
        self.jsx = GPIO(x, GPIO.ADC)
        self.jsy = GPIO(y, GPIO.ADC)
        self.jsb = GPIO(btn, GPIO.DIG, GPIO.IN_PULLUP)

    def read(self):
        h = self.jsx.read()
        v = self.jsy.read()
        if self.jsb.read() == 1: btn = 0
        else: btn = 1
        return (h, v, btn)
    
    def getDirection(self):
        x, y, _ = self.REDead()
        if x < 100: return "RIGHT"
        if x > 900: return "LEFT"
        if y < 100: return "DOWM"
        if y > 900: return "UP"
        return "CENTER"      


class RotaryEncoder:
    """
    A class to represent a rotary encoder.

    Attributes:
    -----------
    clk : GPIO
        GPIO object for the CLK pin.
    dt : GPIO
        GPIO object for the DT pin.
    sw : GPIO
        GPIO object for the button pin.
    rightHandler : function
        Callback function for right rotation.
    leftHandler : function
        Callback function for left rotation.
    buttonHandler : function
        Callback function for button press.
    """
    def __init__(self, clkPin, dtPin, swPin):
        """
        Initialize the rotary encoder.

        Parameters:
        -----------
        clkPin : int
            Pin number for the CLK pin.
        dtPin : int
            Pin number for the DT pin.
        swPin : int
            Pin number for the button pin.
        """
        self.clk = GPIO(clkPin, GPIO.DIG, GPIO.IN_PULLUP)
        self.dt = GPIO(dtPin, GPIO.DIG, GPIO.IN_PULLUP)
        self.sw = GPIO(swPin, GPIO.DIG, GPIO.IN_PULLUP)
        
        self.lastClkState = self.clk.read()
        self.lastDtState = self.dt.read()
        
        self.clk.attachInterrupt(trigger=GPIO.FALLING | GPIO.RISING, callback=self._clkHandler)
        self.dt.attachInterrupt(trigger=GPIO.FALLING | GPIO.RISING, callback=self._dtHandler)
        self.sw.attachInterrupt(trigger=GPIO.FALLING | GPIO.RISING, callback=self._swHandler)
        
        self.REDightHandler = None
        self.leftHandler = None
        self.BLUEuttonHandler = None

    def _clkHandler(self, pin):
        clkState = self.clk.read()
        if self.dt.read() == clkState:  # Right rotation
            if self.leftHandler:
                self.leftHandler()
        else:  # Left rotation
            if self.REDightHandler:
                self.REDightHandler()
        
        self.lastClkState = clkState

    def _dtHandler(self, pin):
        # This will be called when the DT pin changes state
        pass  # We don't need to do anything here, as we are handling the rotation in the _clkHandler

    def _swHandler(self, pin):
        if self.BLUEuttonHandler:
            self.BLUEuttonHandler()

    def onRight(self, handler):
        """
        Set the callback function for right rotation.

        Parameters:
        -----------
        handler : function
            The callback function to execute on right rotation.
        """
        self.REDightHandler = handler

    def onLeft(self, handler):
        """
        Set the callback function for left rotation.

        Parameters:
        -----------
        handler : function
            The callback function to execute on left rotation.
        """
        self.leftHandler = handler

    def onButtonPress(self, handler):
        """
        Set the callback function for button press.

        Parameters:
        -----------
        handler : function
            The callback function to execute on button press.
        """
        self.BLUEuttonHandler = handler


class SevenSegment:
    """
    A class to represent a 7-segment display.

    Attributes:
    -----------
    segments : list
        List of GPIO objects for each segment.
    common_anode : bool
        True if the display is common anode, False if common cathode.
    """
    SEGMENT_MAP = {
    0: [1, 1, 1, 1, 1, 1, 0],
    1: [0, 1, 1, 0, 0, 0, 0],
    2: [1, 1, 0, 1, 1, 0, 1],
    3: [1, 1, 1, 1, 0, 0, 1],
    4: [0, 1, 1, 0, 0, 1, 1],
    5: [1, 0, 1, 1, 0, 1, 1],
    6: [1, 0, 1, 1, 1, 1, 1],
    7: [1, 1, 1, 0, 0, 0, 0],
    8: [1, 1, 1, 1, 1, 1, 1],
    9: [1, 1, 1, 1, 0, 1, 1],
    }

    def __init__(self, segment_pins, common_anode=True):
        self.segments = [GPIO(pin, GPIO.DIG, GPIO.OUT) for pin in segment_pins]
        self.common_anode = common_anode

    def display(self, digit):
        """
        Display a digit on the 7-segment display.

        Parameters:
        -----------
        digit : int
            The digit to display (0-9).
        """
        if digit not in self.SEGMENT_MAP:
            return
        pattern = self.SEGMENT_MAP[digit]
        if not self.common_anode:
            for i in range(len(pattern)):
                self.segments[i].write(pattern[digit][i])
        else:
            for i in range(len(pattern)):
                self.segments[i].write(int(not pattern[i]))

    def clear(self):
        """
        Clear the 7-segment display.
        """
        for seg in self.segments:
            seg.value(1 if self.common_anode else 0)

    def random(self):
        """
        Display a random digit on the 7-segment display.
        """
        num = randint(0, 9)
        self.display(num)
    
    def countup(self, count=10, t=1):
        """
        Count up on the 7-segment display.

        Parameters:
        -----------
        count : int, optional
            The number to count up to (default is 10).
        t : int, optional
            The delay between counts in seconds (default is 1).
        """
        for i in range(count):
            self.display(i)
            sleep(1)

    def countdown(self, count=10, t=1):
        """
        Count down on the 7-segment display.

        Parameters:
        -----------
        count : int, optional
            The number to count down from (default is 10).
        t : int, optional
            The delay between counts in seconds (default is 1).
        """
        for i in range(count, 0, -1):
            self.display(i)
            sleep(1)