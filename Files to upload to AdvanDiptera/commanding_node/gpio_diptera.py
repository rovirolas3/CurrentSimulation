import RPi.GPIO as GPIO

class Rpi_gpio_comm:

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(12, GPIO.OUT)


    def start(self):
        GPIO.output(12, GPIO.HIGH)

if __name__ == "__main__":

    com = Rpi_gpio_comm()
    com.start()
