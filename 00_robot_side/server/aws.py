#AWS server info held here, as well as functions to push and pull data
import os

AWS = os.getenv("AWS_SERVER")
KEY = os.getenv("AWS_KEY")

def ping_server():
    
    return 0


"""
Send:
Robot status, connection status/metrics, latitude, longitude, angle, target location, speed, cam feed(if selected), lookahead x, lookahead y, omega, curvature
For all nav types                                                                                                 | for pure pursuit
"""
def send_data():

    return 0

"""
Pull commands from controller:
Connection status/metrics, Desired robot status, array target points, set speed, [Joystick data if applicable], [Emergency stop command status (y/n)]
"""
def pull_data():

    return 0
