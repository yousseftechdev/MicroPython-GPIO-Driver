from machine import Pin, ADC, PWM
from time import sleep

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

    def __init__(self, pin, pinType="digital", mode="OUT", pwmFreq=1000):
        """
        Initialize the GPIO pin.
        If only the pin number was provided then the pin will be set as digital and output.

        Parameters:
        -----------
        pin : int
            The pin number.
        pinType : str, optional
            The type of the pin (default is "digital").
        mode : str, optional
            The mode of the pin (default is "OUT").
        pwmFreq : int, optional
            The PWM frequency (default is 1000).
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
            self.pin = PWM(pin, pwmFreq)
    
    def flash(self, t=1):
        """
        Flash the digital output pin.
        Default time is 1 second.

        Parameters:
        -----------
        t : int, optional
            The duration to flash the pin in seconds (default is 1).

        Raises:
        -------
        TypeError
            If the pin is not digital and set as OUT.
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

        Raises:
        -------
        TypeError
            If the pin is not digital and set as IN or analog.
        """
        if self.pinType == "analog":
            return self.pin.read_u16()
        elif self.pinType == "digital" and self.mode == "IN":
            return self.pin.value()
        else:
            raise TypeError("Pin has to be either digital and set as IN or analog")
    
    def write(self, state=None, dutyCycle=4095):
        """
        Write a value to the pin.

        Parameters:
        -----------
        state : int, optional
            The state to set the digital pin (0 or 1).
        dutyCycle : int, optional
            The duty cycle for the PWM pin (default is 65535).

        Raises:
        -------
        ValueError
            If the state is not 0 or 1, or if the duty cycle is out of range.
        TypeError
            If the pin is not digital and set as OUT or PWM.
        """
        if self.pinType == "digital" and self.mode == "OUT":
            if state == 0 or state == 1:
                self.pin.value(state)
            else:
                raise ValueError("State has to either be 1 or 0")
        elif self.pinType == "pwm":
            if dutyCycle > 1023 or dutyCycle < 0:
                raise ValueError("Duty cycle must be between 0 and 1023")
            else:
                self.pin.duty(dutyCycle)
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

    def __init__(self, pin, pinType="digital", freq=1000):
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
            self.led = GPIO(pin, pinType="digital", mode="OUT")
        elif pinType == "pwm":
            self.led = GPIO(pin, pinType="pwm", pwmFreq=freq)
    
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
            self.led.write(dutyCycle=1023)
            sleep(t)
            self.led.write(dutyCycle=0)
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
            for i in range(0, 1023, 9):
                self.led.write(dutyCycle=i)
                sleep(0.02)
            self.led.write(dutyCycle=1023)
            
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
            for i in range(1023, 0, -9):
                self.led.write(dutyCycle=i)
                sleep(0.02)
            self.led.write(dutyCycle=0)
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
            self.led.write(dutyCycle=0)
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
            self.led.write(dutyCycle=0)

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
        self.pwm = GPIO(pin, pinType="pwm", pwmFreq=freq)
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
                self.pwm.write(dutyCycle=self._angleToDuty(angle))
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