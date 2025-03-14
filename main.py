from gpio import GPIO
from time import sleep


def main() -> None:
    adc = GPIO(35, pinType="analog")
    digpin = GPIO(12, pinType="digital", mode="IN")
    led = GPIO(14, pinType="pwm")
    led2 = GPIO(4, pinType="digital", mode="OUT")

    print(adc.read())
    print(digpin.read())
    led.write(dutyCycle=10)

    while True:
        led2.toggle()
        sleep(1)
        led2.toggle()
        sleep(1)