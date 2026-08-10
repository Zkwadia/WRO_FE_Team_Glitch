import pigpio

class Servo:
    def __init__(self, pin, frequency=50):
        self.pin = pin
        self.frequency = frequency
        self.pwm = pigpio.pi()
        self.pwm.set_mode(self.pin, pigpio.OUTPUT)
        self.pwm.set_PWM_frequency(self.pin, self.frequency)

    def setAngle(self, angle):
        pulse_width = 500 + round(angle * 11.11)
        self.pwm.set_servo_pulsewidth(self.pin, pulse_width)

    def set_PWM(self, PWM):
        self.pwm.set_PWM_dutycycle(self.pin, PWM)
        
