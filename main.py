from gpio import GPIO, LED, Servo
from time import sleep

def main() -> None:
    adc = GPIO(35, pinType="analog")
    digpin = GPIO(12, pinType="digital", mode="IN")
    led = GPIO(14, pinType="pwm")
    led2 = GPIO(4, pinType="digital", mode="OUT")
    led3 = LED(15, pinType="pwm")
    servo = Servo(16)

    print(adc.read())
    print(digpin.read())
    led.write(dutyCycle=10)

    led3.fadeIn()
    sleep(1)
    led3.fadeOut()

    servo.move(90)
    sleep(1)
    servo.move(0)

    while True:
        led2.toggle()
        sleep(1)
        led2.toggle()
        sleep(1)