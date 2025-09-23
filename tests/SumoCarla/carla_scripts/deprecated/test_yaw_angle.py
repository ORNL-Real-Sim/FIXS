import math
import carla

# Calculate compass bearing between two geographic coordinates (lat/lon pairs)
def calculate_geographic_bearing(start_lat, start_lon, end_lat, end_lon):
    lat1_rad = math.radians(start_lat)
    lat2_rad = math.radians(end_lat)
    delta_lon_rad = math.radians(end_lon - start_lon)
    
    bearing_x = math.sin(delta_lon_rad) * math.cos(lat2_rad)
    bearing_y = math.cos(lat1_rad)*math.sin(lat2_rad) - math.sin(lat1_rad)*math.cos(lat2_rad)*math.cos(delta_lon_rad)
    
    # Convert to degrees and normalize to 0-360 range
    bearing_degrees = math.degrees(math.atan2(bearing_x, bearing_y))
    normalized_bearing = (bearing_degrees + 360) % 360
    
    return normalized_bearing


carla_client = carla.Client('127.0.0.1', 420)
world = carla_client.load_world('Town01')
mymap = world.get_map()

# pick two points along +X
orig_loc = carla.Location(x=0, y=0, z=0)
x1_loc  = carla.Location(x=10, y=0, z=0)    # 10 m out along +X

# convert to geo‐coordinates
orig_geo = mymap.transform_to_geolocation(orig_loc)
x1_geo  = mymap.transform_to_geolocation(x1_loc)

# compute bearing
bearing = calculate_geographic_bearing(
    orig_geo.latitude, orig_geo.longitude,
    x1_geo.latitude,  x1_geo.longitude
)
print(f"+X axis points at a bearing of {bearing:.1f}° (clockwise from true north)")
