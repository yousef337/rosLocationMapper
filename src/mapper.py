#!/usr/bin/env python3
import rospy
import settings
from std_msgs.msg import String
from turtlesim.msg import Pose
from geojson import Feature, Polygon, load
from os import listdir, getcwd

def sendToBot(msg: str):
    pub = rospy.Publisher(settings.botListernNodeName, String, queue_size=10)
    if not rospy.is_shutdown():
        pub.publish(msg)


def getRoomsData():
    """Return dict of names and coordinates of the room"""
    roomsInfo = {}
    geojsonFiles = listdir(getcwd() + settings.geojsondir)
    for file in geojsonFiles:
        with open(f"{getcwd() +settings.geojsondir}/{file}") as f:
            gj = load(f)
            roomsInfo[gj["name"]] = gj['features'][0]['geometry']['coordinates']
    
    return roomsInfo


#https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
def ccw(p1, p2, p3) -> bool:
    return (p3[1]-p1[1]) * (p2[0]-p1[0]) > (p2[1]-p1[1]) * (p3[0]-p1[0])


def intersect(l1, l2):
    return ccw(l1[0], l2[0], l2[1]) != ccw(l1[1], l2[0], l2[1]) and ccw(l1[0], l1[1], l2[0]) != ccw(l1[0], l1[1], l2[1])


def isPointInsidePolygon(point, polygonCords) -> bool:
    orthLine = ((point[0], point[1]), (point[0], point[1] + 10))
    inside: bool = False
    for i in range(len(polygonCords)):
        if intersect(orthLine, ((polygonCords[i]), (polygonCords[(i+1) % len(polygonCords)]))):
            inside = not inside

    return inside


def mapToRoom(location: Pose):
    roomsInfo = getRoomsData()
    for i in roomsInfo.keys():
        if (isPointInsidePolygon((location.x, location.y), roomsInfo[i])):
            print("IN")
            sendToBot(i)
            break

rospy.init_node(settings.nodeName, anonymous=True)
rospy.Subscriber(settings.robotPoseNodeName, Pose, mapToRoom)
rospy.spin()