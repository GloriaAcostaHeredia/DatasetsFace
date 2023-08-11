import serial
import time
import numpy as np
from collections import deque
import warnings
import itertools
import csv


hw_sensor = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
hw_sensor.setDTR(False)
time.sleep(.1)
hw_sensor.flushInput()
hw_sensor.setDTR(True)

label = 'circle'
count_file = 7

M = []
data = []

f = open(f'{label}_{count_file}.csv', 'a')
w = csv.writer(f)

# write the header
w.writerow(['tVal2','tVal3','tVal4','tVal5','tVal6','tVal7','ax','ay','az','gx','gy','gz'])
with hw_sensor:
    while True:
        try:
            line = hw_sensor.readline()
            if not line:
                # HACK: Descartamos líneas vacías porque fromstring produce
                # resultados erróneos, ver https://github.com/numpy/numpy/issues/1714
                continue
            # M.append(np.fromstring(line.decode('ascii', errors='replace'), sep=','))
            # data.append(yy)
            tmp = line.decode('ascii', errors='replace').rstrip().split(',')
            if (len(tmp) >= 5):
                data.append(tmp)

            if (len(data) == 119):
                r = input('¿Guardar?')
                if (r == 's'):
                    for d in data:
                        w.writerow(d)
                    w.writerow([])
                data = []
        except KeyboardInterrupt:
            print("Exiting")
            break
