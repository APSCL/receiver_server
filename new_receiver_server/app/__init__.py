from app.config import DevelopmentConfig, DeploymentConfig, ConfigType
from flask import Flask
from flask_marshmallow import Marshmallow
from flask_sqlalchemy import SQLAlchemy
from flask_migrate import Migrate

# --- imports from old AGV Server
import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request
import requests
import random
import time
import sys
import json
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
from geometry_msgs.msg import PoseStamped

db = SQLAlchemy()
ma = Marshmallow()
migrate = Migrate()

rclpy.init(args=None)
nav = BasicNavigator()
init_pose = PoseStamped()

def create_app(waypoint_ip, conf_type=ConfigType.DEVELOPMENT):
    app = Flask(__name__)
    initialize_config(app, waypoint_ip, conf_type)
    # set up all flask extensions
    initialize_extensions(app)
    # register routes and blueprints
    register_blueprints(app)
    # set up logging within the app to track recieved requests etc
    configure_logging(app)

    return app

def initialize_config(app, waypoint_ip, conf_type):
    conf_dict = {conf.name:conf.value for conf in ConfigType}
    conf_class = conf_dict.get(conf_type.name, DevelopmentConfig)
    app.config.from_object(conf_class)
    app.config["SQLALCHEMY_TRACK_MODIFICATIONS"] = False
    app.config["WAYPOINT_IP_ADDRESS"] = waypoint_ip

def initialize_extensions(app):
    db.init_app(app)
    with app.app_context():
        initialize_database()
    ma.init_app(app)
    migrate.init_app(app, db)

def register_blueprints(app):
    from app.commands import commands
    app.register_blueprint(commands, url_prefix="/commands")
    from app.pings import pings
    app.register_blueprint(pings, url_prefix="/pings")

# TODO: Set up error_handlers, possibly (could just do validation purely from within backend)
def register_error_handlers(app):
    pass

# TODO: Set up logging for the APP
def configure_logging(app):
    pass

def initialize_database():
    # import all models here
    db.create_all()