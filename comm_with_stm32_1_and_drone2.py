# -*- coding: utf-8 -*-
"""
Created on Sun Aug  1 22:48:05 2021

@author: Asus
"""

import serial
import time
import airsim
import numpy as np

SYNC_BYTE = 255
commandDuration = 0.1   # Duration of each command to drone (in seconds)

print("Starting the program")

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.
# Test initial flight
client.takeoffAsync().join()
client.moveToPositionAsync(-10, 10, -10, 5).join()
print("Test flight mission complete")

# Reading coordinates from text file
coords_list = []
with open("output2.txt", "r") as input_fs:
    for line in input_fs:
        contents = line.split(",")
        integers = [int(x) for x in contents]
        coords_list.append(integers)

output_fs_commandInt = open("Command_integer_arrays.txt", "w")
output_fs_commands = open("Commands.txt", "w")

class BBoxCoords:
    x_min = 0
    x_max = 0
    y_min = 0
    y_max = 0

    def __init__(self, x_min, y_min, x_max, y_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max


    def set(self, xy_list):
        self.x_min = xy_list[0]
        self.y_min = xy_list[1]
        self.x_max = xy_list[2]
        self.y_max = xy_list[3]


def send_coords(coords, port):
    dataTx = [0]*9
    dataTx[1], dataTx[0] = divmod(coords.x_min, 256)
    dataTx[3], dataTx[2] = divmod(coords.y_min, 256)
    dataTx[5], dataTx[4] = divmod(coords.x_max, 256)
    dataTx[7], dataTx[6] = divmod(coords.y_max, 256)
    dataTx[8] = SYNC_BYTE
    for i in range(len(dataTx)):
        if i < len(dataTx) - 1 and dataTx[i] == SYNC_BYTE:
            dataTx[i] -= 1
        if dataTx[i] < 0:
            dataTx[i] = 0;

        port.write(bytes([dataTx[i]]))


def receive_integers(port):
    int_array = [0]*5
    for i in range(5):
        x = port.read()
        int_array[i] = int.from_bytes(x, "big")

    return int_array


def moveDrone(droneClient, commandIntArray, duration):
    if commandIntArray[0] == 0:
        # Drone does not move forward
        vx = 0
    elif commandIntArray[0] == 1:
        # Drone moves forward
        vx = 2
    elif commandIntArray[0] == 2:
        # Drone moves backwards
        vx = -2

    if commandIntArray[1] == 0:
        # Drone does not move to the left or right
        vy = 0
    elif commandIntArray[1] == 1:
        # Drone moves to the right
        vy = 2
    elif commandIntArray[1] == 2:
        # Drone moves to the left
        vy = -2

    if commandIntArray[2] == 0:
        # Drone does not move vertically
        vz = 0
    elif commandIntArray[2] == 1:
        # Drone moves down
        vz = 2
    elif commandIntArray[2] == 2:
        # Drone moves up (YES, negative vz value means UP, counterintuitive)
        vz = -2

    if commandIntArray[3] == 0:
        # Drone does not roll
        roll_rate = 0
    elif commandIntArray[3] == 1:
        # Drone rolls to the right
        roll_rate = 5    # in degrees
    elif commandIntArray[3] == 2:
        # Drone rolls to the left
        roll_rate = -5   # in degrees

    droneClient.moveByVelocityAsync(vx, vz, vz, duration).join()
    #droneClient.moveByAngleRatesThrottleAsync(roll_rate*np.pi/180, 0, 0, 1.0,
                                              #duration).join()


    if commandIntArray[4] == 1:
        # Drone lands
        droneClient.landAsync()


def print_commands(commandIntArray):
    commands = [""]*5
    if commandIntArray[0] == 0:
        commands[0] = "0: Drone does not move along the x-axis\n"
    elif commandIntArray[0] == 1:
        commands[0] = "1: Drone moves forward\n"
    elif commandIntArray[0] == 2:
        commands[0] = "2: Drone moves backwards\n"

    print(commands[0])

    if commandIntArray[1] == 0:
        commands[1] = "0: Drone does not move along the y-axis\n"
    elif commandIntArray[1] == 1:
        commands[1] = "1: Drone moves right\n"
    elif commandIntArray[1] == 2:
        commands[1] = "2: Drone moves left\n"

    print(commands[1])

    if commandIntArray[2] == 0:
        commands[2] = "0: Drone does not move vertically\n"
    elif commandIntArray[2] == 1:
        commands[2] = "1: Drone descends\n"
    elif commandIntArray[2] == 2:
        commands[2] = "2: Drone ascends\n"

    print(commands[2])

    if commandIntArray[3] == 0:
        commands[3] = "0: Drone does not roll\n"
    elif commandIntArray[3] == 1:
        commands[3] = "1: Drone rolls right\n"
    elif commandIntArray[3] == 2:
        commands[3] = "2: Drone rolls left\n"

    print(commands[3])

    if commandIntArray[4] == 0:
        commands[4] = "0: Drone does not land\n"
    elif commandIntArray[4] == 1:
        commands[4] = "1: Drone lands\n"

    print(commands[4])

    print("\n")
    return commands


bbcoords = BBoxCoords(0, 0, 0, 0)
commandIntList = []

# Communication with STM32
ser = serial.Serial('com3', 9600, timeout=None, bytesize=8)
#print(ser.name)
print("connected to: " + ser.portstr)

for idx in range(len(coords_list)):
    # Set coordinates in object
    bbcoords.set(coords_list[idx])

    # Send coordinates to STM32
    send_coords(bbcoords, ser)

    # Receive command integer array (as dataRx_int) from STM32
    dataRx_int = receive_integers(ser)
    print(dataRx_int)
    commands_list = print_commands(dataRx_int)
    output_fs_commands.writelines(commands_list)
    output_fs_commands.write("\n")
    output_fs_commandInt.write(str(dataRx_int))
    output_fs_commandInt.write("\n")

    # Move the drone as per the command integer array
    moveDrone(client, dataRx_int, commandDuration)
    commandIntList.append(dataRx_int)


print("Flight mission complete")

ser.close()
print('Serial communication closed')
output_fs_commands.close()
output_fs_commandInt.close()
print("Program terminated")
