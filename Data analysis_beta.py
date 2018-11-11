# -*- coding: utf-8 -*-
"""
Created on Thu Oct  4 08:09:20 2018

@author: 14625
"""
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("C:\\Users\\14625\\OneDrive\\Documents\\GitHub\\DataAcquire\\test_11_7_inside_doorclose_1.txt", delimiter = ',', skiprows = 2, skipfooter = 1,engine = 'python')
#data.to_csv("D:\\Physics398DLP\\Testfile_1.csv")
data = data.values
#data_3=np.genfromtxt("C:\\Users\\14625\\OneDrive\\Documents\\GitHub\\DataAcquire\\test_10_17_2018_3.txt", delimiter = ',', skip_header = 4, skip_footer = 1)

count = int(len(data)/3)

time_hour = np.zeros(count)

time_min = np.zeros(count)
time_sec = np.zeros(count)
time_ms = np.zeros(count)
temperature = np.zeros(count)
pressure = np.zeros(count)
humidity = np.zeros(count)
altitude = np.zeros(count)
seconds = np.zeros(count)
anemometer = np.zeros(count)
d_temperature = np.zeros(count)


for n in range(0,len(data)):
    if n%3 == 0:
        time_hour[int(n/3)] = (data[n,0])
        time_min[int(n/3)] = (data[n,1])
        time_sec[int(n/3)] = (data[n,2])
        time_ms[int(n/3)] = (data[n,3])
    elif n%3 == 1:
        temperature[int((n-1)/3)] = (data[n,0])
        pressure[int((n-1)/3)] = (data[n,1])
        humidity[int((n-1)/3)] = (data[n,2])
        altitude[int((n-1)/3)] = (data[n,3])
    else:
        anemometer[int((n-2)/3)] = data[n,0]
        

t0 = time_hour[0]*3600 + time_min[0]*60 + time_sec[0] + time_ms[0]*0.001
init_time = np.linspace(t0,t0,count)
seconds = time_hour*3600 + time_min*60 + time_sec + time_ms*0.001 - init_time
anemometer = (anemometer * 2 / 1023 - 0.4) / 1.6 * 32.4

#for n in range(1,count):
#    d_temperature[n] = (temperature[n]-temperature[n-1])/(seconds[n]-seconds[n-1])


raw_data = {'time': seconds, 
        'temperature': temperature, 
        'pressure': pressure, 
        'humidity': humidity,
        'anemometer': anemometer}
df = pd.DataFrame(raw_data, columns = ['time', 'temperature', 'pressure', 'humidity', 'anemometer'])
df.to_csv("D:\\Physics398DLP\\Testfile_1.csv")

    

       

 

plt.figure(figsize=(15,10))
plt.plot(seconds,temperature, 'y')
plt.plot(seconds,d_temperature, 'b')
#plt.plot(seconds_2,temperature_2, 'r')
plt.title('Temperature(°C) vs Time')
plt.xlabel('time(s)')
plt.ylabel('Temperature(°C)')
plt.show()

plt.figure(figsize=(15,10))
plt.plot(seconds,pressure, 'm')
#plt.plot(seconds_2,pressure_2, 'r')
plt.title('Pressure(hPa) vs Time')
plt.xlabel('time(s)')
plt.ylabel('Pressure(hPa)')
plt.show()

plt.figure(figsize=(15,10))
plt.plot(seconds,humidity, 'c')
#plt.plot(seconds_2,humidity_2, 'b')
plt.title('Humidity(%) vs Time')
plt.xlabel('time(s)')
plt.ylabel('Humidity(%)')
plt.show()

plt.figure(figsize=(15,10))
plt.plot(seconds,anemometer, 'b')
#plt.plot(seconds_2,humidity_2, 'b')
plt.title('Windspeed(m/s) vs Time')
plt.xlabel('time(s)')
plt.ylabel('Windspeed(m/s)')
plt.show()
#plt.plot(seconds,altitude, 'k')
#plt.title('Altitude(m) vs Time')
#plt.xlabel('t')
#plt.ylabel('m')
#plt.show()





# syncronize clocks different devices
# measure settling time for humidity, etc.
# anamometer
# pcb and bredboard ready
# case design

# check the heater on bme
# test IR sensor
# still anamometer
# still settling time