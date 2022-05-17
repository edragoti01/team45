#!/usr/bin/env python3

#TASK NOTES/PLAN

# 3 minute limit

#9 equal sized zones

#4 rooms - speed rather than number of rooms

#Minimise incidents

#Arena will be different
#May not always start in the same place

#ADVANCED FEATURE
#Pic of beacon - always red, green, blue, or yellow

#Colour selected using command line argument
#(SEE LAUNCH FILES (ADVANCED) SECTION ON WIKI)


#ADVANCED FEATURE
#Mapping with SLAM

#Maps room in background whilst other tasks are completed
#(SEE LAUNCH FILES (ADVANCED) SECTION ON WIKI)



#CONSTANTS
#9 zones
#4 rooms
#Red, green, blue, yellow beacons (not sure if there will always be 4)


#HOW SHOULD THE ROBOT EXPLORE?
#Sense environment
#IF OK: forward
#ELIF: left
#ELIF: right
#ELSE: backwards

#OR

#Calculate furthest distance
#Turn and move this way until it meets an object

#####IF 3 LEFT TURNS OR 3 RIGHT TURNS - GO OPPOSITE (AVOID LOOPS)

####USE BOARD LENGTHS TO HELP SENSE ROOMS?

####IF LEFT OR RIGHT ARC DISTANCE < WHATEVER, TURN THAT WAY 
#Helps find rooms
#Should take priority in if statement