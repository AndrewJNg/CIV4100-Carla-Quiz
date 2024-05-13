#!/usr/bin/env python

import glob
import os
import sys
import random
import time
import argparse
import math
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

def find_actor_by_rolename(world,
                           role_name_tofind):  # searches all actors in a carla world looking for a specific role name and returns object if found
    actors = world.get_actors()
    actors = actors.filter('vehicle.*')  # filter out just the vehicle actors

    if (actors):
        for actor in actors:
            role_name = "None"
            if 'role_name' in actor.attributes:
                if (actor.attributes['role_name'] == role_name_tofind):
                    return actor
        return None
    else:
        return None


def calc_dist(actor_a, actor_b):  # calculates distance in xy plane between two actors
    loc_a = actor_a.get_location()
    loc_b = actor_b.get_location()
    return math.sqrt((loc_a.x - loc_b.x) ** 2 + (loc_a.y - loc_b.y) ** 2 + (loc_a.z - loc_b.z) ** 2)

X_ego = -2.1
Y_ego = 90
Z_ego = 0.2

PITCH = 0
YAW = 90
ROLL = 0

EGO_VEHICLE_NAME = 'ego_vehicle'
EGO_VEHICLE_MODEL ='vehicle.tesla.model3'
EGO_VEHICLE_VELOCITY = 3

LEAD_VEHICLE_NAME = 'lead_vehicle'
SAFETY_DIST = 15

client = carla.Client('localhost', 2000)  # create client to connect to simulator
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()


ego_vehicle_bp = next(bp for bp in blueprint_library if bp.id == EGO_VEHICLE_MODEL)
ego_vehicle_bp.set_attribute('role_name', EGO_VEHICLE_NAME)

ego_spawn_loc = carla.Location(X_ego, Y_ego, Z_ego)
ego_rotation = carla.Rotation(PITCH, YAW, ROLL)
ego_transform = carla.Transform(ego_spawn_loc, ego_rotation)

ego_vehicle = world.spawn_actor(ego_vehicle_bp, ego_transform)
lead_vehicle = find_actor_by_rolename(world, LEAD_VEHICLE_NAME)


time.sleep(5)
ego_vehicle.set_target_velocity(carla.Vector3D(0, EGO_VEHICLE_VELOCITY, 0))

while True:
    # safe for the ego vehicle to move
    while (calc_dist(lead_vehicle, ego_vehicle) >= SAFETY_DIST):
        try:
            print("Safe to move")
        except KeyboardInterrupt:
            lead_vehicle.destroy()
            ego_vehicle.destroy()

    # Stop the car        
    ego_vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))

    # swap lanes
       
    
    # ego vehicle pass the safety distance, stop the ego vehicle
    # while (calc_dist(lead_vehicle, ego_vehicle) < SAFETY_DIST):
    #     try:
    # 	    print("Waiting for lead vehcile to move")
    #     except KeyboardInterrupt:
    #         lead_vehicle.destroy()
    #         ego_vehicle.destroy()
    time.sleep(1)
    ego_vehicle.set_target_velocity(carla.Vector3D(0, EGO_VEHICLE_VELOCITY, 0))
            
lead_vehicle.destroy()
ego_vehicle.destroy()