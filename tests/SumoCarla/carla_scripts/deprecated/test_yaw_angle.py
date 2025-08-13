import math
import carla

# helper to compute bearing from (lat1,lon1) → (lat2,lon2)
def geo_bearing(lat1, lon1, lat2, lon2):
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    Δλ = math.radians(lon2 - lon1)
    x = math.sin(Δλ) * math.cos(φ2)
    y = math.cos(φ1)*math.sin(φ2) - math.sin(φ1)*math.cos(φ2)*math.cos(Δλ)
    return (math.degrees(math.atan2(x, y)) + 360) % 360

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
bearing = geo_bearing(
    orig_geo.latitude, orig_geo.longitude,
    x1_geo.latitude,  x1_geo.longitude
)
print(f"+X axis points at a bearing of {bearing:.1f}° (clockwise from true north)")
