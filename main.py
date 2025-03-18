from gpio import GPIO, LED, Servo, Stepper, StepperULN, UltraSonic, Joystick
from time import sleep

def main() -> None:
    adc = GPIO(35, GPIO.ADC)
    digPin = GPIO(12, GPIO.DIG, GPIO.IN)
    led = GPIO(14, GPIO.PWM)
    led2 = GPIO(4, GPIO.DIG, GPIO.OUT)
    led3 = LED(15, GPIO.PWM)
    servo = Servo(16)
    stepper = Stepper(17, 18, 19)
    ultraSonic = UltraSonic(20, 21)
    joystick = Joystick(22, 23, 24)
    stepperULN = StepperULN(25, 26, 27, 28, delay=5, mode=StepperULN.HALFSTEP)

    print(adc.read())
    print(digPin.read())
    led.write(dutyCycle=10)

    led3.fadeIn()
    sleep(1)
    led3.fadeOut()

    servo.move(90)
    sleep(1)
    servo.move(0)

    stepper.powerOn()
    stepper.steps(200)
    stepper.powerOff()

    print(f"Distance: {ultraSonic.getDistanceCm()} cm")

    stepperULN.angle(90)
    sleep(1)
    stepperULN.angle(-90)
    stepperULN.reset()

    while True:
        h, v, btn = joystick.read()
        print(f"Joystick: X={h}, Y={v}, Button={btn}")
        led2.toggle()
        sleep(1)