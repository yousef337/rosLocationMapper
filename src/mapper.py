#!/usr/bin/env python3
import rospy
import settings
from std_msgs.msg import String
from turtlesim.msg import Pose
from geojson import Feature, Polygon, load
from os import listdir, getcwd
from .srv import locationMapper, locationMapperResponse, locationMapperRequest


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


def mapToRoom(location: locationMapperRequest) -> locationMapperResponse:
    response: locationMapperResponse
    roomsInfo = getRoomsData()

    for i in roomsInfo.keys():
        if (isPointInsidePolygon((location.x, location.y), roomsInfo[i])):
            response.room = i
            return response

    response.room = "Not Found"
    return response


rospy.init_node(settings.serviceName, anonymous=True)
mapper = rospy.Service(settings.serviceName, locationMapper, mapToRoom)
rospy.spin()