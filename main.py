from gpio import GPIO, LED, Servo, Stepper, StepperULN, UltraSonic, Joystick, SevenSegment, RotaryEncoder
from time import sleep

def main() -> None:
    # Initialize components
    adc = GPIO(35, GPIO.ADC)
    digPin = GPIO(12, GPIO.DIG, GPIO.IN)
    led = LED(14, GPIO.PWM)
    led2 = GPIO(4, GPIO.DIG, GPIO.OUT)
    servo = Servo(16)
    stepper = Stepper(17, 18, 19)
    stepperULN = StepperULN(25, 26, 27, 28, delay=5, mode=StepperULN.HALFSTEP)
    ultraSonic = UltraSonic(20, 21)
    joystick = Joystick(22, 23, 24)
    seven_seg = SevenSegment([2, 3, 4, 5, 6, 7, 8], common_anode=True)
    rotary_encoder = RotaryEncoder(29, 30, 31)

    # GPIO examples
    print(f"ADC Value: {adc.read()}")
    print(f"Digital Pin Value: {digPin.read()}")
    led.flash(1)

    # LED examples
    led.fadeIn()
    sleep(1)
    led.fadeOut()

    # Servo example
    servo.move(90)
    sleep(1)
    servo.move(0)

    # Stepper motor example
    stepper.powerOn()
    stepper.steps(200)
    stepper.powerOff()

    # StepperULN example
    stepperULN.angle(90)
    sleep(1)
    stepperULN.angle(-90)
    stepperULN.reset()

    # Ultrasonic sensor example
    print(f"Distance: {ultraSonic.get_distance_cm()} cm")

    # Joystick example
    h, v, btn = joystick.read()
    print(f"Joystick: X={h}, Y={v}, Button={btn}")

    # Seven-segment display example
    seven_seg.display(5)
    sleep(1)
    seven_seg.clear()

    # RotaryEncoder example
    def on_right():
        print("Rotary Encoder turned right")

    def on_left():
        print("Rotary Encoder turned left")

    def on_button():
        print("Rotary Encoder button pressed")

    rotary_encoder.onRight(on_right)
    rotary_encoder.onLeft(on_left)
    rotary_encoder.onButtonPress(on_button)

    # Infinite loop for joystick and LED toggle
    while True:
        h, v, btn = joystick.read()
        print(f"Joystick: X={h}, Y={v}, Button={btn}")
        led2.toggle()
        sleep(1)