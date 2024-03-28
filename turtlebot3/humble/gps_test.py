from pyproj import Transformer

class Location:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class RobotState:
    def __init__(self, location: Location):
        self.location = location

class GPSConversionTool:
    def __init__(self, frame: str):
        crs_code = {'svy21': 'EPSG:3414', 'korea_map': 'EPSG:5179'}
        if frame not in crs_code:
            raise Exception("Unsupported coordinate frame")

        self._transformer_to_wgs = Transformer.from_crs(crs_code[frame], 'EPSG:4326')
        self._transformer_from_wgs = Transformer.from_crs('EPSG:4326', crs_code[frame])

    def robot_state_to_gps(self, robot_state: RobotState):
        lat, lon = self._transformer_to_wgs.transform(robot_state.location.y, robot_state.location.x)
        return lat, lon

    def gps_to_robot_state(self, latitude: float, longitude: float):
        x, y = self._transformer_from_wgs.transform(latitude, longitude)
        return RobotState(Location(x, y))

# # Example usage
# converter = GPSConversionTool('svy21')

# # Converting robot state to GPS coordinates
# robot_state = RobotState(Location(22000, 31500))
# latitude, longitude = converter.robot_state_to_gps(robot_state)
# print(f"Robot State to GPS: Latitude = {latitude}, Longitude = {longitude}")

# # Converting GPS coordinates to RobotState
# robot_state = converter.gps_to_robot_state(latitude, longitude)
# print(f"GPS to Robot State: X = {robot_state.location.x}, Y = {robot_state.location.y}")


# Example usage
converter = GPSConversionTool('korea_map')

# Converting GPS coordinates to RobotState
latitude=37.607790
longitude=126.891300
robot_state = converter.gps_to_robot_state(latitude, longitude)
print(f"GPS to Robot State: X = {robot_state.location.x}, Y = {robot_state.location.y}")

# Converting robot state to GPS coordinates
robot_state = RobotState(Location(robot_state.location.y, robot_state.location.x))
latitude, longitude = converter.robot_state_to_gps(robot_state)
print(f"Robot State to GPS: Latitude = {latitude}, Longitude = {longitude}")
