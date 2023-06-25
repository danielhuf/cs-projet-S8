#! /usr/bin/env python3

import os
import rospy
from client import set_joint_position, move_gripper


while True:
    ########## COMPUTER VISION ##########
     
    # Calculate trajectory
    # publish service request
    
    th1 = 1.0

    ########## SET POSITION ##########
    grab_pos = [0.0, -0.1, 0.55, 1.0]
    base_pos = [0.0, 0.0, 0.0, 0.0]
    
    set_joint_position(grab_pos[0], grab_pos[1], grab_pos[2], grab_pos[3], 2)
    move_gripper("close")
    set_joint_position(th1, base_pos[1], base_pos[2], base_pos[3], 2)


    ########## MATLAB COMMUNICATION ##########
    # send coordinates to matlab

    move_gripper("open")

    ########## Loop ##########
    set_joint_position(base_pos[0], base_pos[1], base_pos[2], base_pos[3], 2)

    throw_again = input("Throw_again? (Y/N) ")

    while throw_again not in ("N", "n", "Y", "y"):
        throw_again = input("Invalid input. Try again. (Y/N) ")

    if throw_again in ("N", "n"):
        break

    x = eval(input("Chose new cup coordinates: \nx: "))
    y = eval(input("y: "))


    ########## Respawn models ##########
    
    os.system("rosservice call gazebo/delete_model '{model_name: ball}'")
    os.system("roslaunch open_manipulator_gazebo spawn_ball.launch")

    os.system("rosservice call gazebo/delete_model '{model_name: cup}'")
    os.system("roslaunch open_manipulator_gazebo spawn_cup.launch x:=%s y:=%s" %(x, y))


