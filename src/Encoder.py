import pigpio
import RPi.GPIO as GPIO
import time
import math
#import board
from math import atan2, sqrt
#import matplotlib.pyplot as plt
import multiprocessing


class EncoderCounter:
    channelA = 9
    channelB = 11
    const = (2 * math.pi * 2)

    def __init__(self):

        self.pi = pigpio.pi()

        self.pi.set_mode(self.channelA, pigpio.INPUT)
        self.pi.set_pull_up_down(self.channelA, pigpio.PUD_UP)  # channel A
        self.pi.set_mode(self.channelB, pigpio.INPUT)
        self.pi.set_pull_up_down(self.channelB, pigpio.PUD_UP)  # channel B

        self.prev_state_A = self.pi.read(self.channelA)
        self.prev_state_B = self.pi.read(self.channelB)
        self.count = 0
        self.x = 0
        self.y = 0
        self.dx = 0
        self.dy = 0
        self.error_x = 0
        self.error_y = 0
        self.x_history = [self.x]
        self.y_history = [self.y]
        self.prev_distance = 0



    def get_position(self, heading, counter):
        # with self.lock:
        # revolution = (self.prev_count - self.count)/1040

        revolution = (counter/2015)  #168
        distance_cm = revolution * self.const
        change = distance_cm - self.prev_distance
        self.dx = math.cos(math.radians(heading)) * change
        self.dy = math.sin(math.radians(heading)) * change
        # print(f"Change : {change}")

        self.x += self.dx
        self.y += self.dy

        self.prev_distance = distance_cm

        # self.prev_count = self.count
        return self.x, self.y



