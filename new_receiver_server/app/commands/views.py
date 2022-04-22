from app import db
from .models import TestModel
from flask import make_response, request, current_app, jsonify
from flask_restful import Resource
from .serializers import TestModelSchema
from . import commands_api

# --- (Imports from old AGV Server)
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

# imports from global scope related to ROS NAV commander
from app import nav



class TestView(Resource):
    def get(self):
        # second_pose = PoseStamped()
        # second_pose.header.stamp = nav.get_clock().now().to_msg()
        # second_pose.pose.position.x = 1.0
        # second_pose.pose.position.y = 3.0
        # second_pose.pose.orientation.z = 1.0
        # second_pose.pose.orientation.w = 0.0
        # second_pose.header.frame_id = 'map'
        # nav.goToPose(second_pose)
        # print("I AM HERE", second_pose)
        print("received")
        return make_response(jsonify("request received"), 200)

    def post(self):
        print("received")
        return make_response(jsonify("Not_implemented"), 400)
        
        # errors = TestModelSchema().validate(request.form)
        # if errors:
        #     return make_response(jsonify(errors) , 400)
        # test_model = TestModelSchema().load(request.form)
        # create_test_model(db, test_model)
        # return make_response(jsonify(TestModelSchema().dump(test_model)), 201)
        

# TODO - create a utils.py file for db model creation
def create_test_model(db, test_model):
    db.session.add(test_model)
    db.session.commit()  

commands_api.add_resource(TestView, '/test_route')