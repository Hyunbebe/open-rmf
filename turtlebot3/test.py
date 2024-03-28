import roslibpy
import time


ros = roslibpy.Ros(host='192.168.50.210', port=9090)
ros.on_ready(lambda: print('Is ROS connected?', ros.is_connected))
ros.run()

print(ros.get_topics())
print(ros.get_topic_type('/ublox/fix'))

def print_message(message):
    print("Latitude: {}".format(message['latitude']))
    print("Longitude: {}".format(message['longitude']))
    print("Altitude: {}".format(message['altitude']))
    # Add more fields as neede

listener = roslibpy.Topic(ros, 'ublox/fix', 'sensor_msgs/NavSatFix')
print('go')

listener.subscribe(print_message)

try:
    while True:
        pass
except KeyboardInterrupt:
    ros.terminate()