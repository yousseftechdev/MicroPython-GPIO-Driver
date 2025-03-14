from machine import Pin, ADC, PWM
from time import sleep

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
    
    def write(self, state=None, dutyCycle=65535):
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
            if dutyCycle > 65535 or dutyCycle < 0:
                raise ValueError("Duty cycle must be between 0 and 65535")
            else:
                self.pin.duty_u16(dutyCycle)
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