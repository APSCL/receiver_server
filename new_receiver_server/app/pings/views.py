import json
from flask import request, Response, jsonify
from flask_restful import Resource
from flask_api import status
from . import pings_api

class TestConnectionView(Resource):
    def get(self):
        return Response(
            response=jsonify({"message":"request received"}),
            status=status.HTTP_200_OK
        )

pings_api.add_resource(TestConnectionView, "/test_connection/")