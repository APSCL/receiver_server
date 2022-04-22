from enum import Enum
import os
from dotenv import load_dotenv

load_dotenv()

class DevelopmentConfig:
    SECRET_KEY = os.environ.get("SECRET_KEY", None)
    SQLALCHEMY_DATABASE_URI = os.environ.get("SQLALCHEMY_DATABASE_URI", "sqlite:///agv.db")
    PORT = int(os.environ.get("PORT", 5000))
    # HOST = os.environ.get("HOST", "0.0.0.0")
    # TODO: Change later after development
    HOST = "0.0.0.0"

# Config for Deployed Sever
class DeploymentConfig:
    SECRET_KEY = os.environ.get("SECRET_KEY", None)
    SQLALCHEMY_DATABASE_URI = os.environ.get("SQLALCHEMY_DATABASE_URI", "sqlite:///agv.db")
    PORT = int(os.environ.get("PORT", 5000))
    HOST = "0.0.0.0"

class ConfigType(Enum):
    DEVELOPMENT = DevelopmentConfig
    DEPLOYMENT = DeploymentConfig
