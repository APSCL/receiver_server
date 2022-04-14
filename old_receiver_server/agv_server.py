import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import String
import threading
from flask import Flask, request
import requests
import random
import time
import sys
import json
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped


rclpy.init(args=None)
app = Flask(__name__)

#setup nav2 basic navigator:
nav = BasicNavigator()

init_pose = PoseStamped()
init_pose.header.stamp = nav.get_clock().now().to_msg()
init_pose.pose.position.x = 0.0
init_pose.pose.position.y = 2.0
init_pose.pose.orientation.z = 1.0
init_pose.pose.orientation.w = 0.0
init_pose.header.frame_id = 'map'

print('setting initial pose')
nav.setInitialPose(init_pose)
print('starting nav2 lifecycle')
nav.lifecycleStartup()
print('got past lifecycleStartup')

# function uses basicNavigator to tell Nav2 to follow the given waypoints
def publish_nav2(waypoints):
    #cancel any existing navigation attempts first:
    nav.cancelNav()

    print('attempting following waypoints')
    nav.followWaypoints(waypoints)
    # ros2_node.publish_message("-1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'")
    print('got past followWaypoints')
    startTime = time.perf_counter()

    navSucceeded = False
    while not navSucceeded:

        while not nav.isNavComplete():
            #update_location(True)
            feedback = nav.getFeedback()
            print('this is feedback:', str(feedback))
            max_duration = 30 #seconds
            if time.perf_counter() - startTime > max_duration:
                current_waypoint = feedback.current_waypoint
                print('got up until waypoint number', str(current_waypoint))
                nav.cancelNav()
                print(max_duration, 'seconds elapsed, repeating attempt to follow waypoints')
                waypoints = waypoints[current_waypoint:] # only attempt waypoints that hadn't completed yet
                nav.followWaypoints(waypoints) 
                time.sleep(1) #give time so isNavComplete can realize a new nav is being attempted
                startTime = time.perf_counter()

        print('got past isNavComplete')

        result = nav.getResult()
        print('this is navResult:', str(result))

        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!')
            navSucceeded = True # ends outer loop and returns

        else:
            print('Goal did not succeed, trying again...')
            current_waypoint = feedback.current_waypoint
            print('got up until waypoint number', str(current_waypoint))
            nav.cancelNav()
            waypoints = waypoints[current_waypoint:] # only attempt waypoints that hadn't completed yet
            nav.followWaypoints(waypoints)
            time.sleep(1) #give time so isNavComplete can realize a new nav is being attempted
            startTime = time.perf_counter()
            # outer loop will repeat since navSucceeded==False

    return result

def convertLatLonXY(lat, lon):
    # TODO
    return float(lat), float(lon)

# receive waypoints from server and call publish_nav2
@app.route('/receiveWaypoints', methods=('GET','POST'))
def receive_waypoints():
    if request.method == 'POST':

        data = request.form
        print('got this raw data:', data)

        waypoints = []
        for i in range(len(data) // 2):
            print("getting lat/lon number", i)
            lat, lon = data['latitude'+str(i)], data['longitude'+str(i)]
            x, y = convertLatLonXY(lat, lon)
            print('got x,y: ', x, y)
            waypoint = PoseStamped()
            waypoint.pose.position.x, waypoint.pose.position.y = x, y
            #waypoint.pose.orientation.z = 1.0 #, waypoint.pose.orientation.w = 1.0, 0.0
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = nav.get_clock().now().to_msg()
            waypoints.append(waypoint)

        navResult = publish_nav2(waypoints)
    
    return None


@app.route('/cancelNav', methods=('GET','POST'))
def cancel_nav():  
    if request.method == 'GET':
        print('cancelling nav...')
        nav.cancelNav()
    return "cancelled"


#TODO this function needs to get location of AGV in coordinates
def update_location(shouldRespond):
    if shouldRespond:
        #TODO get location of AGV, possibly from echoing nav2 topics?
        #lat, lon = getLocationOfAGV()
        lat, lon = random.randint(-90,90), random.randint(-90,90)
        coordinates = {'latitude':lat, 'longitude':lon}
        address = "http://localhost:5000/agvlocation"
        res = requests.post(url=address, data=coordinates)
        #TODO have a way for it to stop sending its location when done

        threading.Timer(1, update_location, [True]).start() # runs every 1 second
        
    return

def getLocationOfAGV():
    #TODO
    return


#####################################################################################################
# Different architecture version: agv server requests waypoints

# @app.route('/getFromServer')
# def follow_waypoints():
#     received_data = get_from_server()
#     print("This is received_data:", received_data)

#     #TODO loop through all received waypoints
#     waypoints = []
#     waypointString = ""
#     for i in range(len(received_data) // 2):
#         print("getting lat/lon number", i)
#         lat, lon = received_data['latitude'+str(i)], received_data['longitude'+str(i)]
#         x, y = convertLatLonXY(lat, lon)
#         waypointString += "(" + str(x) + "," + str(y) +") \n"
#         print('got x,y: ', x, y)
#         waypoint = PoseStamped()
#         waypoint.pose.position.x, waypoint.pose.position.y = x, y
#         #waypoint.pose.orientation.z = 1.0 #, waypoint.pose.orientation.w = 1.0, 0.0
#         waypoint.header.frame_id = 'map'
#         waypoint.header.stamp = nav.get_clock().now().to_msg()
#         waypoints.append(waypoint)

#     navResult = publish_nav2(waypoints)
#     msg = "received the following waypoints from waypoint server:\n" + waypointString + "\n Navigation result: "
#     if navResult:
#         msg += str(navResult)
#     else:
#         msg += "'None'"
#     return msg

# def get_from_server():
#     addressToGet = "http://0.0.0.0:5000/getwaypoint" #adjust this to be dummy server's IP
#     res = requests.get(addressToGet)
#     data = res.json()
#     print('data received from server:', data) #, file=sys.stdout)
#     #ros2_node.publish_message(json.dumps(data))
#     update_location(shouldRespond=False)
#     return data
    
