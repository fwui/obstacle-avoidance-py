# Obstacle Avoidance System for SUAS Written in py
# March 24, 2023

import argparse
import time
#import os
#import rospy
from pymavlink import mavutil

DISTANCE_TO_MAINTAIN = 3

# Args and Parser ( Mungkin perlu diubah ini )
parser = argparse.ArgumentParser(description='Obstacle avoidance for drone')
parser.add_argument('--baudrate', type=int, default=57600, help='Serial baud rate')
parser.add_argument('--device', type=str, default='/dev/ttyACM0', help='Serial device path')
args = parser.parse_args()

# Connection Settings
master = mavutil.mavlink_connection(args.device, baud=args.baudrate)

# System Initialisation
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_ENABLE_OBSTACLE_AVOIDANCE,
    0, 1, 0, 0, 0, 0, 0, 0)

# Command: Set the altitude to maintain
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_HOME,
    0, 0, 0, 0, 0, 0, 10)

# Command: Set Velocity to Fly
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
    0, 0, 5, -1, 0, 0, 0)

# Condition can be modified in the future but for now, Let's assume that its always active
while True:
    # Check for obstacles within <DISTANCE_TO_MAINTAIN> meters of the drone
    msg = master.recv_match(type='OBSTACLE_DISTANCE', blocking=True)
    if msg and msg.distance < DISTANCE_TO_MAINTAIN:
        print("Obstacle detected!")
        # Stop the drone and wait for 5 seconds before continuing
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0)
        time.sleep(10)
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0)

    # Fly if no obstacle found
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, 0, 5, 0, 0, 0, 0)
    # master.mav.command_long_send(
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_CMD_REVERT_SPEED,
    #     0, 0, 0, 5, 0, 0, 0)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    time.sleep(10)