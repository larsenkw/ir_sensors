#!/usr/bin/env python

# This is the node for processing the commands sent by the phone to the Arduino
# using Bluetooth. The phone sends a command to the Arduino which then sends a
# message to a command topic which this node reads and can then process based on
# the command name and its value.

from __future__ import print_function
import rospy
from ir_sensors.msg import Command
import csv

def checkUsername(username):
    # #FIXME: send userfound message to confirm process workds
    # command_response.name = "checkuser"
    # command_response.value1 = "userfound"

    # Open .csv database and check for username in file
    # Columns: username,displayname,password
    command_response = Command()
    command_response.name = "checkuser"
    command_response.value1 = "usernotfound"
    command_response.value2 = ""
    database = open('./database.csv', 'rb')
    database_reader = csv.reader(database, delimiter = ',')
    for row in database_reader:
        if (row[0] == username):
            command_response.value1 = "userfound"
            break

    pub_command_response.publish(command_response)
    print("You called the checkUsername function.")

def checkPassword(username, password):
    # #FIXME: send passwordcorrect message to confirm process works
    # command_response.name = "checkpassword"
    # command_response.value1 = "correct"
    # command_response.value2 = "0" # number of tries left

    # Open .csv database and check if password is correct
    # If correct send back "correct" as value1 and [displayname] as value2
    command_response = Command()
    command_response.name = "checkpassword"
    command_response.value1 = "incorrect"
    command_response.value2 = ""
    database = open('./database.csv', 'rb')
    database_reader = csv.reader(database, delimiter = ',')
    for row in database_reader:
        if (row[0] == username):
            if (row[2] == password):
                command_response.value1 = "correct"
                command_response.value2 = row[1] # this is the displayname

    pub_command_response.publish(command_response)
    print("you called the checkPassword function.")

def addUsername(username):
    pass

def addDisplayName(username, displayname):
    pass

def addPassword(username, password):
    pass

def callback_bt_command(msg):
    if (msg.name == "checkuser"):
        # First value should be the username
        checkUsername(msg.value1)
    elif (msg.name == "checkpassword"):
        # First value username, second value password
        checkPassword(msg.value1, msg.value2)

def bt_command_process():
    # Initialize ROS Node and Subscriber
    rospy.init_node("bt_command_process")
    rospy.Subscriber("bt_command", Command, callback_bt_command)
    rospy.spin()

# ROS Publisher for sending response to Arduino
pub_command_response = rospy.Publisher("bt_command_response", Command, queue_size=10)

if __name__ == "__main__":
    try:
        bt_command_process()
    except rospy.ROSInterruptException:
        pass
