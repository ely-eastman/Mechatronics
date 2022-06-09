# read data from the imu and plot

# sudo apt-get install python3-pip
# python -m pip install pyserial
# sudo apt-get install python-matplotlib

import serial
import time
import math
ser = serial.Serial('COM4',230400)
print('Opening port: ')
print(ser.name)
alpha = 0.95

read_samples = 10 # anything bigger than 1 to start out
ax = []
ay = []
az = []
gx = []
gy = []
gz = []
temp = []
times = []
print('Requesting data collection...')
ser.write(b'\n')
prev_time = time.time()
while read_samples > 1:
    data_read = ser.read_until(b'\n',200) # get the data as bytes
    data_text = str(data_read,'utf-8') # turn the bytes to a string
    data = [float(i) for i in data_text.split()] # turn the string into a list of floats

    if(len(data)==8):
        read_samples = int(data[0]) # keep reading until this becomes 1
        ax.append(data[1])
        ay.append(data[2])
        az.append(data[3])
        gx.append(data[4])
        gy.append(data[5])
        gz.append(data[6])
        temp.append(data[7])
        times.append(time.time()-prev_time)
        prev_time = time.time()
print('Data collection complete')
# plot it
import matplotlib.pyplot as plt 
t = range(len(ax)) # time array
plt.plot(t,ax,'r*-',t,ay,'b*-',t,az,'k*-')
plt.ylabel('G value')
plt.xlabel('sample')
plt.show()

t = range(len(gx)) # time array
plt.plot(t,gx,'r*-',t,gy,'b*-',t,gz,'k*-')
plt.ylabel('Omega value')
plt.xlabel('sample')
plt.show()

t = range(len(temp)) # time array
plt.plot(t,temp,'r*-')
plt.ylabel('Temperature value')
plt.xlabel('sample')
plt.show()

#writing the complementary filter (ish)
#first we want a table of the times!
int_times = []
for i in range(len(times)):
    int_times.append(sum(times[:i+1:1]))
#now we want a tables of the integrated data. Each data list _should_ be the same length so this should work:
int_ax = []
int_ay = []
int_az = []
int_gx = []
int_gy = []
int_gz = []
total_times = []
for i in range(len(ax)):
    int_ax.append(sum([a*t for a,t in zip(ax[:i+1],times[:i+1])]))
    int_ay.append(sum([a*t for a,t in zip(ay[:i+1],times[:i+1])]))
    int_az.append(sum([a*t for a,t in zip(az[:i+1],times[:i+1])]))
    int_gx.append(sum([g*t for g,t in zip(gx[:i+1],times[:i+1])]))
    int_gy.append(sum([g*t for g,t in zip(gy[:i+1],times[:i+1])]))
    int_gz.append(sum([g*t for g,t in zip(gz[:i+1],times[:i+1])]))
    total_times.append(sum(times[:i+1]))
#now we have all the variables integrated. We need to convert to pitch and yaw. This is just gx, gy, and the values computed by the accelerations.

gpitch = int_gx
gyaw = int_gy
#need to use lambdas for the accelerations
apitch = list(map(lambda ax,az,ay: math.atan2(ay, (ax**2+az**2)**0.5) * 180/math.pi,int_ax,int_ay,int_az))
ayaw = list(map(lambda ax,az: math.atan2(-ax,az)*180/math.pi,int_ax,int_az))
#now to sum the variables in the filter

filtered_pitch = list(map(lambda gp,ap: alpha * gp + (1-alpha)*ap,gpitch,apitch))
filtered_yaw = list(map(lambda gy,ay: alpha * gy + (1-alpha)*ay,gyaw,ayaw))

#finally, plot the two variables
t=range(len(filtered_pitch))
plt.plot(total_times,filtered_pitch,'r*-',total_times,filtered_yaw,'b*-')
plt.xlabel('Time')
plt.ylabel('Angle')
plt.legend(['Pitch','Yaw'])
plt.show()



#now we just need to combine the two variables into the filter.

# be sure to close the port
ser.close()