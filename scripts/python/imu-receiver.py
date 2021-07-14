# Author: Raul de Santos Rico
# Description: Script that receives data from the IMU sensor through a YARP port and saves it to a CSV
# CopyPolicy: released under the terms of the LGPLv2.1
# Python version: 2.7

from time import sleep
import csv
import yarp # imports YARP

DELAY = 0.01 # IMU sensor reading period 
csvFile = "imu.csv"

# YARP
yarp.Network.init()  # connect to YARP network

if yarp.Network.checkNetwork() != True:  # let's see if there was actually a reachable YARP network
    print('[error] Please try running yarp server')  # tell the user to start one with 'yarp server' if there isn't any
    quit()

#create a new input port and open it
imu_port = yarp.Port()
imu_port.open("/imu/in")
#connect up the output port to our input port
yarp.Network.connect("/softimu/out", "/imu/in")

# writing CSV
start = yarp.now()
imu = yarp.Vector(2)

with open(csvFile, 'w') as csvOutfile:
    csvwriter = csv.writer(csvOutfile, delimiter=',')
    csvwriter.writerow(['timestamp', 'Imu-roll', 'Imu-pitch'])
    while(1):
        imu_port.read(imu)
        print('timestam: ', round(yarp.now() - start, 3))
        print('Imu: [', imu[0],', ',imu[1], ']')
        csvwriter.writerow([round(yarp.now() - start, 3), imu[0], imu[1]])
