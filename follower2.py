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

def calc_dist2(actor_a, loc_b):  # calculates distance in xy plane between two actors
    loc_a = actor_a.get_location()
    return math.sqrt((loc_a.x - loc_b.x) ** 2 + (loc_a.y - loc_b.y) ** 2 + (loc_a.z - loc_b.z) ** 2)
  
MAX_STEERING_ANGLE = 70

def convert_to_carla_steer(desired_steer, max_angle=MAX_STEERING_ANGLE):
  """
  Converts desired steering angle (in degrees) to a value compatible with CARLA controls.

  Args:
      desired_steer: The desired steering angle in degrees (between -max_angle and max_angle).
      max_angle: The maximum steering angle of the car (default: 70.0).

  Returns:
      A float value between -1.0 and 1.0 suitable for CARLA's steering control.
  """
  # Clamp desired angle to valid range
  desired_steer = max(-max_angle, min(desired_steer, max_angle))

  # Convert to CARLA range (-1.0 to 1.0)
  carla_steer = desired_steer / max_angle
  return carla_steer
   
def goto_coordinate(ego_vehicle, target_location):
  """
  This function navigates the ego vehicle to a specified target location.

  Args:
      ego_vehicle: The carla.Actor object representing the ego vehicle.
      target_location: A carla.Location object representing the desired destination.
  """
  while True:
    current_location = ego_vehicle.get_location()
    #print(current_location)
    #print(target_location)
    distance = calc_dist2(ego_vehicle, target_location)

    # Stop if close enough to the target
    if distance < 2.0:  # Adjust threshold based on your needs
      print("Reached target location!")
      ego_vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
      break

    # Calculate direction vector
    direction = target_location - current_location
    direction /= math.sqrt(direction.x**2 + direction.y**2 + direction.z**2)  # Normalize

    # Adjust steering (replace with your car's steering control logic)
    #new_steer = math.atan2(direction.y, direction.x) * 180 / math.pi
    new_steer = math.atan(direction.y/direction.x) * 180 / math.pi
    #ego_vehicle.apply_control(carla.Control(steer=new_steer))
    #ego_vehicle.set_wheel_steer_direction(angle_in_deg=new_steer)
    #control = carla.VehicleControl(steer=new_steer, throttle=1.0)  # Adjust throttle as needed
    #ego_vehicle.apply_control(control)
    #ego_vehicle.VehicleAckermannControl(steer=new_steer)
    
    #steer_val = -new_steer/MAX_STEERING_ANGLE
    #print(new_steer/MAX_STEERING_ANGLE)
    #steer_val = -convert_to_carla_steer(new_steer)
    #print(steer_val)
    #control = carla.VehicleControl(steer=steer_val*math.pi/180)  # Adjust throttle as needed
    #ego_vehicle.apply_control(control)
    steer_val = -new_steer/70
    print(steer_val)
    control = carla.VehicleControl(steer=steer_val)  # Adjust throttle as needed
    ego_vehicle.apply_control(control)
    

    # Maintain speed
    ego_vehicle.set_target_velocity(carla.Vector3D(0, EGO_VEHICLE_VELOCITY, 0))

    time.sleep(0.1)  # Adjust update rate based on your simulation
    
  

# Apply steering control with conversion
#control = carla.VehicleControl(steer=convert_to_carla_steer(new_steer))
#ego_vehicle.apply_control(control)


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

    # Example usage:
    print(lead_vehicle.get_location())
    lead_loc = lead_vehicle.get_location()
    
    print(lead_loc.x+3)
    target_location = carla.Location(lead_loc.x+3, lead_loc.y, lead_loc.z) # Modify coordinates as needed
    goto_coordinate(ego_vehicle, target_location)
    # Stop the car        
    
    #ego_vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
    
    #'''
    target_location = carla.Location(lead_loc.x, lead_loc.y+10, lead_loc.z) # Modify coordinates as needed
    goto_coordinate(ego_vehicle, target_location)
    # Stop the car        
    
    target_location = carla.Location(lead_loc.x, lead_loc.y+30, lead_loc.z) # Modify coordinates as needed
    goto_coordinate(ego_vehicle, target_location)
    # Stop the car     
    ego_vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
    #'''

    # swap lanes
       
    
    # ego vehicle pass the safety distance, stop the ego vehicle
    # while (calc_dist(lead_vehicle, ego_vehicle) < SAFETY_DIST):
    #     try:
    # 	    print("Waiting for lead vehcile to move")
    #     except KeyboardInterrupt:
    #         lead_vehicle.destroy()
    #         ego_vehicle.destroy()
    time.sleep(1)
    #ego_vehicle.set_target_velocity(carla.Vector3D(0, EGO_VEHICLE_VELOCITY, 0))
            
lead_vehicle.destroy()
ego_vehicle.destroy()