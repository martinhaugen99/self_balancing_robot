import serial
import matplotlib.pyplot as plt
from collections import deque
import time

ser = serial.Serial('/dev/tty.usbmodem11303', 115200)
time.sleep(2)
ser.flushInput()

buffer_size = 200
acc_data = deque(maxlen=buffer_size)
gyro_data = deque(maxlen=buffer_size)
kalman_data = deque(maxlen=buffer_size)

plt.ion()
fig, ax = plt.subplots()

line1, = ax.plot([], [], label="Accel")
line2, = ax.plot([], [], label="Gyro")
line3, = ax.plot([], [], label="Kalman")
ax.legend()
ax.set_ylim(-90, 90)

while True:
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(line)
        # if line.startswith("Acc:"):
        #     parts = line.replace("Acc:", "").split(",")
        #     acc = float(parts[0])
        #     gyro = float(parts[1].replace("Gyro:", ""))
        #     kalman = float(parts[2].replace("KF:", ""))

        #     acc_data.append(acc)
        #     gyro_data.append(gyro)
        #     kalman_data.append(kalman)

        #     line1.set_data(range(len(acc_data)), acc_data)
        #     line2.set_data(range(len(gyro_data)), gyro_data)
        #     line3.set_data(range(len(kalman_data)), kalman_data)
        #     ax.set_xlim(0, buffer_size)
        #     plt.pause(0.01)

    except Exception as e:
        print("Error:", e)
        break
