#!/usr/bin/env python3
"""
sim_gps.py -- latitude/longitude for points in the IGVC sim worlds.

The igvc*.sdf worlds pin Gazebo's world origin to WORLD_ORIGIN below
(<spherical_coordinates>, ENU, heading 0: world +x = east, +y = north). The
location is Oakland University, Rochester MI, where IGVC is held; it is
approximate and only has to be self-consistent with the worlds.

Stdlib only. Converts with the WGS84 ellipsoid's local radii, which is what
Gazebo uses and is exact to millimetres over the few hundred metres here.

    python3 tools/sim_gps.py                 # datum + every sim waypoint
    python3 tools/sim_gps.py 15.2 30.0       # any world x y

Keep these numbers in step with:
  src/esda_simulation_2025/worlds/igvc*.sdf        <spherical_coordinates>
  src/esda_simulation_2025/config/navsat_sim.yaml  datum (robot spawn)
  src/esda_simulation_2025/waypoints/igvc_waypoints.yaml  latitude/longitude
"""
import math
import os
import sys

WORLD_ORIGIN = (42.668086, -83.218507)    # lat, lon (deg) of world (0, 0)
SPAWN = (11.0, 0.0)                       # igvc* worlds spawn the robot here, yaw 0

A = 6378137.0                             # WGS84 semi-major axis (m)
E2 = 6.69437999014e-3                     # WGS84 first eccentricity squared


def radii(lat_deg):
    """(meridional, prime-vertical) radii of curvature at a latitude."""
    s = math.sin(math.radians(lat_deg))
    w = math.sqrt(1.0 - E2 * s * s)
    return A * (1.0 - E2) / w ** 3, A / w


def world_to_latlon(x, y, origin=WORLD_ORIGIN):
    """World ENU metres (x east, y north) -> (lat, lon) degrees."""
    m, n = radii(origin[0])
    lat = origin[0] + math.degrees(y / m)
    lon = origin[1] + math.degrees(x / (n * math.cos(math.radians(origin[0]))))
    return lat, lon


def latlon_to_world(lat, lon, origin=WORLD_ORIGIN):
    """Inverse of world_to_latlon."""
    m, n = radii(origin[0])
    y = math.radians(lat - origin[0]) * m
    x = math.radians(lon - origin[1]) * n * math.cos(math.radians(origin[0]))
    return x, y


def main(argv):
    if len(argv) == 2:
        lat, lon = world_to_latlon(float(argv[0]), float(argv[1]))
        print(f'{lat:.8f} {lon:.8f}')
        return 0
    lat, lon = world_to_latlon(*SPAWN)
    print(f'datum (robot spawn, world {SPAWN}): [{lat:.8f}, {lon:.8f}, 0.0]')
    import yaml
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src',
                        'esda_simulation_2025', 'waypoints', 'igvc_waypoints.yaml')
    for wp in yaml.safe_load(open(path))['waypoints']:
        # waypoint x/y are in the map frame, whose origin is the spawn pose.
        wx = SPAWN[0] + wp['position']['x']
        wy = SPAWN[1] + wp['position']['y']
        lat, lon = world_to_latlon(wx, wy)
        print(f"{wp['name']:22s} map ({wp['position']['x']:7.2f}, {wp['position']['y']:6.2f})"
              f" -> {lat:.8f}, {lon:.8f}")
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
