from gpio import GPIO, LED, Servo, Stepper, StepperULN, UltraSonic, Joystick
from time import sleep

def main() -> None:
    adc = GPIO(35, GPIO.ADC)
    digpin = GPIO(12, GPIO.DIG, GPIO.IN)
    led = GPIO(14, GPIO.PWM)
    led2 = GPIO(4, GPIO.DIG, GPIO.OUT)
    led3 = LED(15, GPIO.PWM)
    servo = Servo(16)
    stepper = Stepper(17, 18, 19)
    ultrasonic = UltraSonic(20, 21)
    joystick = Joystick(22, 23, 24)
    stepper_uln = StepperULN(25, 26, 27, 28, delay=5, mode=StepperULN.HALFSTEP)

    print(adc.read())
    print(digpin.read())
    led.write(dutyCycle=10)

    led3.fadeIn()
    sleep(1)
    led3.fadeOut()

    servo.move(90)
    sleep(1)
    servo.move(0)

    stepper.power_on()
    stepper.steps(200)
    stepper.power_off()

    print(f"Distance: {ultrasonic.get_distance_cm()} cm")

    stepper_uln.angle(90)
    sleep(1)
    stepper_uln.angle(-90)
    stepper_uln.reset()

    while True:
        h, v, btn = joystick.read()
        print(f"Joystick: X={h}, Y={v}, Button={btn}")
        led2.toggle()
        sleep(1)