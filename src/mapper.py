#!/usr/bin/env python3
import rospy
import settings
from geojson import Feature, Polygon, load
from math import sin, cos
from os import listdir, getcwd
from locationMapper.srv import locationMapper, locationMapperResponse, locationMapperRequest, roomMapper, roomMapperResponse, roomMapperRequest, doorMapper, doorMapperResponse, doorMapperRequest
from statistics import mean 
import angles
from scipy.spatial.transform import Rotation

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
    orthLine = ((point[0], point[1]), (point[0], point[1] + settings.POLYGON_CORNOR_FRAME))
    inside: bool = False
    for i in range(len(polygonCords)):
        if intersect(orthLine, ((polygonCords[i]), (polygonCords[(i+1) % len(polygonCords)]))):
            inside = not inside

    return inside


def mapToRoom(location: locationMapperRequest) -> locationMapperResponse:
    response: locationMapperResponse = locationMapperResponse()
    roomsInfo = getRoomsData()

    for i in roomsInfo.keys():
        if (isPointInsidePolygon((location.point.x, location.point.y), roomsInfo[i])):
            response.room = i
            return response

    response.room = "Not Found"
    return response


def mapToLoc(request: roomMapperRequest) -> roomMapperResponse:
    response: roomMapperResponse = roomMapperResponse()
    roomInfo = getRoomsData()[request.room]

    if not roomInfo:
        return response

    response.point.x = mean(map(lambda l: l[0] , roomInfo))
    response.point.y = mean(map(lambda l: l[1] , roomInfo))

    return response

def getDoorSegment(room: str):
    geojsonFiles = listdir(getcwd() + settings.geojsondir)
    for file in geojsonFiles:
        with open(f"{getcwd() + settings.geojsondir}/{file}") as f:
            gj = load(f)
            if gj["name"] == room and gj["features"][0]["geometry"]["type"] == "Line":
                return (gj['features'][0]['geometry']['coordinates'][0], gj['features'][0]['geometry']['coordinates'][1])
            
def midPoint(line):
    return ((line[0][0] - line[1][0]) / 2, (line[0][1] - line[1][1]) / 2)

def distance(p1, p2):
    return sqrt(pow(p1[0]-p2[0], 2) + pow(p1[1]-p2[1], 2))

def doorToMove(request: doorMapperRequest) -> doorMapperResponse:
    response: doorMapperResponse = doorMapperResponse()

    quatRotation = Rotation.from_quat(request.orientation)
    eulerRotation = quatRotation.as_euler('xyz', degrees=False)
    projTheta = eulerRotation[2]

    pose = (request.position.x, request.position.y)
    poseProj = (pose[0] + settings.VISION_RANGE * cos(projTheta), pose[1] + settings.VISION_RANGE * sin(projTheta))
    
    doorCords = getDoorSegment(request.room)

    if doorCords == None or not intersect(poseProj, doorCords):
        response.degree = -999
        return response

    # check if the point of intersection is within the feasible range, if yes, return 0, else return the degree to turn to face the center
    
    mProj = (poseProj[1] - pose[1]) / (poseProj[0] - poseProj[0])
    mDoor = (doorCords[0][1] - doorCords[1][1]) / (doorCords[0][0] - doorCords[0][1])
    bProj = poseProj[1] - mProj * poseProj[0]
    bDoor = doorCords[0][1] - mDoor * doorCords[0][0]

    intersectionPoint = ((bProj - bDoor) / (mDoor - mProj), 0)
    intersectionPoint[1] = mDoor * intersectionPoint[0] + bDoor
    
    if abs(distance(midPoint(doorCords), intersectionPoint)) < settings.feasibleSegment:
        response.degree = 0
        return response
    else:
        diff = (midPoint(doorCords)[0] - pose[0], midPoint(doorCords)[1] - pose[1])
        diffTheta = atan2(diff[1], diff[0])
        response.degree = angles.shortest_angular_distance(projTheta, diffTheta)
        return response

    return response

rospy.init_node(settings.serviceName, anonymous=True)
locMapper = rospy.Service(settings.locationServiceName, locationMapper, mapToRoom)
roomMapper = rospy.Service(settings.roomServiceName, roomMapper, mapToLoc)
rospy.spin()