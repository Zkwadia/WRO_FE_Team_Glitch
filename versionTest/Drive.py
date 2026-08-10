import unittest
import pigpio

class Drive:
    def __int__(self, pwm, power):
        self.power = power
        self.pwm = pwm
        self.pwm.set_mode(12, pigpio.OUTPUT)  # Set pin 12 as an output
        self.pwm.set_mode(20, pigpio.OUTPUT)  # Set pin 20 as an output
        self.pwm.hardware_PWM(12, 100, 0)
        self.previous_power = 0
    def forward(self, power):
        self.pwm.set_PWM_dutycycle(12, 2.55 * self.power)
        self.pwm.write(20, 1)  # Set pin 20 high
    def reverse(self, power):
        self.pwm.set_PWM_dutycycle(12, power)
        self.pwm.write(20, 1)  # Set pin 20 high
    def stop(self):
        self.pwm.set_PWM_dutycycle(12, 0)

if __name__ == '__main__':
    pwm = pigpio.pi()
    power = 60
    drive = Drive(pwm, power)
    prev_power = 0
    while True:
        total_power = (power * 0.1) + (prev_power * 0.9)
        prev_power = total_power
        drive.forward(total_power)
