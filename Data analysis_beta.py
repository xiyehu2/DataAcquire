# -*- coding: utf-8 -*-
"""
Created on Thu Oct  4 08:09:20 2018

@author: 14625
"""

import numpy as np
import pandas as pd
import scipy.stats
import matplotlib.pyplot as plt

file_name = 'test_11_7_inside_dooropen_1.txt'

data = pd.read_csv("C:\\Users\\14625\\OneDrive\\Documents\\GitHub\\DataAcquire\\byThePlants3.txt", delimiter = ',', skiprows = 2, skipfooter = 1,engine = 'python')
data_2 = pd.read_csv("C:\\Users\\14625\\OneDrive\\Documents\\GitHub\\DataAcquire\\byThePlants3.txt", delimiter = ',', skiprows = 2, skipfooter = 1,engine = 'python')
data_3 = pd.read_csv("C:\\Users\\14625\\OneDrive\\Documents\\GitHub\\DataAcquire\\" + file_name, delimiter = ',', skiprows = 2, skipfooter = 1,engine = 'python')
data = data.values
data_2 = data_2.values
data_3 = data_3.values
#data_3=np.genfromtxt("C:\\Users\\14625\\OneDrive\\Documents\\GitHub\\DataAcquire\\byTheFronDoor.txt", delimiter = ',', skip_header = 4, skip_footer = 1)

count = int(len(data)/3)
##----------------------------------##
count_2 = int(len(data_2)/3)

count_3 = int(len(data_3)/3)


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
number = np.linspace(1,count,count)
##---------------------------------------------##
time_hour_2 = np.zeros(count_2)
time_min_2 = np.zeros(count_2)
time_sec_2 = np.zeros(count_2)
time_ms_2 = np.zeros(count_2)
temperature_2 = np.zeros(count_2)
pressure_2 = np.zeros(count_2)
humidity_2 = np.zeros(count_2)
altitude_2 = np.zeros(count_2)
seconds_2 = np.zeros(count_2)
anemometer_2 = np.zeros(count_2)
d_temperature_2 = np.zeros(count_2)
number_2 = np.linspace(1,count_2,count_2)
##---------------------------------------------##
time_hour_3 = np.zeros(count_3)
time_min_3 = np.zeros(count_3)
time_sec_3 = np.zeros(count_3)
time_ms_3 = np.zeros(count_3)
temperature_3 = np.zeros(count_3)
pressure_3 = np.zeros(count_3)
humidity_3 = np.zeros(count_3)
altitude_3 = np.zeros(count_3)
seconds_3 = np.zeros(count_3)
anemometer_3 = np.zeros(count_3)
d_temperature_3 = np.zeros(count_3)
number_3 = np.linspace(1,count_3,count_3)

##------------------



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


for i in time_ms:
    if i < 0:
        i = 1000 + i
    if i < 10:
        i = 100 * i
    if i < 100:
        i = 10 * i
##---------------------------------------------------##        
for m in range(0,len(data_2)):
    if m%3 == 0:
        time_hour_2[int(m/3)] = (data_2[m,0])
        time_min_2[int(m/3)] = (data_2[m,1])
        time_sec_2[int(m/3)] = (data_2[m,2])
        time_ms_2[int(m/3)] = (data_2[m,3])
    elif m%3 == 1:
        temperature_2[int((m-1)/3)] = (data_2[m,0])
        pressure_2[int((m-1)/3)] = (data_2[m,1])
        humidity_2[int((m-1)/3)] = (data_2[m,2])
        altitude_2[int((m-1)/3)] = (data_2[m,3])
    else:
        anemometer_2[int((m-2)/3)] = data_2[m,0]


for i in time_ms_2:
    if i < 0:
        i = 1000 + i
    if i < 10:
        i = 100 * i
    if i < 100:
        i = 10 * i
##---------------------------------------------------##        
for m in range(0,len(data_3)):
    if m%3 == 0:
        time_hour_3[int(m/3)] = (data_3[m,0])
        time_min_3[int(m/3)] = (data_3[m,1])
        time_sec_3[int(m/3)] = (data_3[m,2])
        time_ms_3[int(m/3)] = (data_3[m,3])
    elif m%3 == 1:
        temperature_3[int((m-1)/3)] = (data_3[m,0])
        pressure_3[int((m-1)/3)] = (data_3[m,1])
        humidity_3[int((m-1)/3)] = (data_3[m,2])
        altitude_3[int((m-1)/3)] = (data_3[m,3])
    else:
        anemometer_3[int((m-2)/3)] = data_3[m,0]


for i in time_ms_3:
    if i < 0:
        i = 1000 + i
    if i < 10:
        i = 100 * i
    if i < 100:
        i = 10 * i
       

 
raw_data = {'time': seconds_3, 
        'temperature': temperature_3, 
        'pressure': pressure_3, 
        'humidity': humidity_3,
        'anemometer': anemometer_3}
df = pd.DataFrame(raw_data, columns = ['time', 'temperature', 'pressure', 'humidity', 'anemometer'])
df.to_csv("D:\\Physics398DLP\\Testfile_1.csv")



t1 = time_hour[0]*3600 + time_min[0]*60 + time_sec[0] + time_ms[0]*0.001
t2 = time_hour_2[0]*3600 + time_min_2[0]*60 + time_sec_2[0] + time_ms_2[0]*0.001
t0 = min(t1,t2)
seconds = time_hour*3600 + time_min*60 + time_sec + time_ms*0.001 - np.linspace(t0,t0,count)
seconds_2  = time_hour_2*3600 + time_min_2*60 + time_sec_2 + time_ms_2*0.001 - np.linspace(t0,t0,count_2)
seconds_3 = time_hour_3*3600 + time_min_3*60 + time_sec_3 + time_ms_3*0.001
anemometer = (anemometer - 0.4) / 1.6 * 32.4
anemometer_2 = (anemometer_2 - 0.4) / 1.6 * 32.4
anemometer_3 = (anemometer_3*2/1023 - 0.4) / 1.6 * 32.4


#for n in range(1,count):
#    d_temperature[n] = (temperature[n]-temperature[n-1])/(seconds[n]-seconds[n-1])
x = pressure_3

y = temperature_3




slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(x, y)




#p30 = np.poly1d(np.polyfit(x, y, 30))
plt.figure(figsize=(15,10))
#plt.plot(number,seconds, 'c')
#plt.plot(number_2,seconds_2, 'm')
plt.plot(number_3,seconds_3, 'y')
plt.title('Time Increment')
plt.xlabel('Index')
plt.ylabel('Time(s)')
plt.show() 

plt.figure(figsize=(15,10))
#plt.plot(seconds,temperature, 'c')
#plt.plot(seconds_2,temperature_2, 'm')
plt.plot(seconds_3,temperature_3, 'y')
plt.title('Temperature(°C) vs Time')
plt.xlabel('time(s)')
plt.ylabel('Temperature(°C)')
plt.show()

plt.figure(figsize=(15,10))
#plt.plot(seconds,pressure, 'c')
#plt.plot(seconds_2,pressure_2, 'm')
plt.plot(seconds_3,pressure_3, 'y')
plt.title('Pressure(hPa) vs Time')
plt.xlabel('time(s)')
plt.ylabel('Pressure(hPa)')
plt.show()

plt.figure(figsize=(15,10))
#plt.plot(seconds,humidity, 'c')
#plt.plot(seconds_2,humidity_2, 'm')
plt.plot(seconds_3,humidity_3, 'y')
plt.title('Humidity(%) vs Time')
plt.xlabel('time(s)')
plt.ylabel('Humidity(%)')
plt.show()

plt.figure(figsize=(15,10))
#plt.plot(seconds,anemometer, 'c')
#plt.plot(seconds_2,anemometer_2, 'm')
plt.plot(seconds_3,anemometer_3, 'y')
plt.title('Windspeed(m/s) vs Time')
plt.xlabel('time(s)')
plt.ylabel('Windspeed(m/s)')
plt.show()

#plt.figure(figsize=(15,10))
#plt.scatter(pressure,anemometer)
#plt.title('Windspeed(m/s) vs Pressure(hPa)')
#plt.xlabel('hPa')
#plt.ylabel('m/s')
#plt.show()

#plt.figure(figsize=(15,10))
#plt.scatter(pressure_2,anemometer_2)
#plt.title('Windspeed(m/s) vs Pressure(hPa)')
#plt.xlabel('hPa')
#plt.ylabel('m/s')
#plt.show()

plt.figure(figsize=(15,10))
plt.plot(x,y, 'y')
plt.plot(x,intercept + slope*x,'r')
plt.title(file_name + '    Temperature(°C) vs Pressure(hPa)   '+' Linear fit ' + '   R^2 = ' + str(r_value**2))
plt.xlabel('hPa')
plt.ylabel('°C')
plt.show()




#plt.plot(x, y, '.', x, p30(x), '--')

#plt.figure(figsize=(15,10))
#plt.plot(np.unique(pressure_3), np.poly1d(np.polyfit(pressure_3, anemometer_3, 1))(np.unique(pressure_3)))
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



## another data taking  
## Temp difference in and outside the case
## Report