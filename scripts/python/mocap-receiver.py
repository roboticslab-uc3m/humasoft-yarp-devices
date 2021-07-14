# Author: Raul de Santos Rico
# Description: Script that receives data from the MOCAP sensor through a YARP port and saves it to a CSV
# CopyPolicy: released under the terms of the LGPLv2.1
# Python version: 2.7

from time import sleep
import csv
import yarp # imports YARP

DELAY = 0.01 # IMU sensor reading period 
csvFile = "mocap.csv"

# YARP
yarp.Network.init()  # connect to YARP network

if yarp.Network.checkNetwork() != True:  # let's see if there was actually a reachable YARP network
    print('[error] Please try running yarp server')  # tell the user to start one with 'yarp server' if there isn't any
    quit()

#create a new input port and open it
moc_port = yarp.Port()
moc_port.open("/softmocap/in")
#connect up the output port to our input port
yarp.Network.connect("/softmocap/out", "/softmocap/in")

# writing CSV
start = yarp.now()
mocap = yarp.Vector(2)
imu = yarp.Vector(2)

with open(csvFile, 'w') as csvOutfile:
    csvwriter = csv.writer(csvOutfile, delimiter=',')
    csvwriter.writerow(['timestamp', 'Mocap-roll', 'Mocap-pitch'])
    while(1):
        moc_port.read(mocap)
        print('timestam: ', round(yarp.now() - start, 3))
        print('Mocap: [', mocap[0],', ',mocap[1], ']')
        csvwriter.writerow([round(yarp.now() - start, 3), mocap[0], mocap[1]])
