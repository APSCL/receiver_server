from app import create_app
import sys
from app.config import ConfigType
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--run_type", type=str, required=False)
parser.add_argument("--waypoint_ip", type=str, required=True)
args = parser.parse_args()

def retrieve_config(args):
    if not args.run_type: 
        return ConfigType.DEVELOPMENT
    config_type_dict = {
        'dev':ConfigType.DEVELOPMENT, 
        'deploy':ConfigType.DEPLOYMENT, 
    }
    config_type = config_type_dict.get(args.run_type, None)
    if not config_type:
        sys.exit("[ERROR] - select a valid run_type: [dev deploy test] in order to run server")
    return config_type

application = create_app(args.waypoint_ip, conf_type=retrieve_config(args))

if __name__ == "__main__":
    application.run(host=application.config["HOST"], port=application.config["PORT"], debug=True)
