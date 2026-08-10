# -*- coding: utf-8 -*-

import os
import pigpio
import time
import multiprocessing
class TFmini:
    def __init__(self, RX_Head, RX_Left, RX_Right, RX_Back, baudrate=115200 ):
        self.RX_Head = RX_Head
        self.RX_Left = RX_Left
        self.RX_Right = RX_Right
        self.RX_Back = RX_Back
        # SUJAL START
        #SUJAL END
        
        #self.lock = lock or multiprocessing.Lock()

        self.pi = pigpio.pi()
        self.pi.set_mode(self.RX_Head, pigpio.INPUT)
        self.pi.set_mode(self.RX_Left, pigpio.INPUT)
        self.pi.set_mode(self.RX_Right, pigpio.INPUT)
        self.pi.set_mode(self.RX_Back, pigpio.INPUT)

        self.pi.bb_serial_read_open(self.RX_Head, baudrate)
        self.pi.bb_serial_read_open(self.RX_Left, baudrate)
        self.pi.bb_serial_read_open(self.RX_Right, baudrate)
        self.pi.bb_serial_read_open(self.RX_Back, baudrate)

        self.distance_head = 0
        self.distance_left = 0
        self.distance_right = 0
        self.distance_back = 0

    def getTFminiData(self):
        #with self.lock:
            time.sleep(0.01)  # Adjust the sleep if needed
            count_head, recv_head = self.pi.bb_serial_read(self.RX_Head)
            count_left, recv_left = self.pi.bb_serial_read(self.RX_Left)
            count_right, recv_right = self.pi.bb_serial_read(self.RX_Right)
            count_back, recv_back = self.pi.bb_serial_read(self.RX_Back)
            try:
                if count_head > 8:
                    for i in range(0, count_head - 9):
                        if recv_head[i] == 89 and recv_head[i + 1] == 89:  # 0x59 is 89
                            checksum = 0
                            for j in range(0, 8):
                                checksum += recv_head[i + j]
                            checksum %= 256
                            if checksum == recv_head[i + 8]:
                                self.distance_head = recv_head[i + 2] + recv_head[i + 3] * 256

                if count_left > 8:
                    for i in range(0, count_left - 9):
                        if recv_left[i] == 89 and recv_left[i + 1] == 89:  # 0x59 is 89
                            checksum = 0
                            for j in range(0, 8):
                                checksum += recv_left[i + j]
                            checksum %= 256
                            if checksum == recv_left[i + 8]:
                                self.distance_left = recv_left[i + 2] + recv_left[i + 3] * 256

                if count_right > 8:
                    for i in range(0, count_right - 9):
                        if recv_right[i] == 89 and recv_right[i + 1] == 89:  # 0x59 is 89
                            checksum = 0
                            for j in range(0, 8):
                                checksum += recv_right[i + j]
                            checksum %= 256
                            if checksum == recv_right[i + 8]:
                                self.distance_right = recv_right[i + 2] + recv_right[i + 3] * 256
                                
                if count_back > 8:
                    for i in range(0, count_back - 9):
                        if recv_back[i] == 89 and recv_back[i + 1] == 89:  # 0x59 is 89
                            checksum = 0
                            for j in range(0, 8):
                                checksum += recv_back[i + j]
                            checksum %= 256
                            if checksum == recv_back[i + 8]:
                                self.distance_back = recv_back[i + 2] + recv_back[i + 3] * 256

                return self.distance_head, self.distance_left, self.distance_right, self.distance_back
            except Exception as e:
                print(f"Exception:{e}")

    def close(self):
        self.pi.bb_serial_read_close(self.RX_Head)
        self.pi.bb_serial_read_close(self.RX_Left)
        self.pi.bb_serial_read_close(self.RX_Right)
        self.pi.bb_serial_read_close(self.RX_Back)
        self.pi.stop()

