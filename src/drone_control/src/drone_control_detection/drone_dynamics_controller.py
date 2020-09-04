#!/usr/bin/env python

import time
import math
import argparse
import rospy
from drone_control.msg import TagLocation

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='')
args = parser.parse_args()

# --------------------------------------------------
# -------------- PARAMETERS
# --------------------------------------------------
rad_2_deg = 180.0 / math.pi
deg_2_rad = 1.0 / rad_2_deg

land_alt_cm = 50.0
angle_descend = 20 * deg_2_rad
land_speed_cms = 30.0


# --------------------------------------------------
# -------------- FUNCTIONS
# --------------------------------------------------
class DroneDynamicsControl:

    def __init__(self):

        self.time_0 = time.time()
        self.freq_send = 1

        self._tag_location_sub = rospy.Subscriber('/location/tag', TagLocation, queue_size=1,
                                                  callback=self.localise_and_land)

    @staticmethod
    def get_location_metres(self, original_location, dNorth, dEast):

        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

        print "dlat, dlon", dLat, dLon

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.lon + (dLon * 180 / math.pi)
        return (newlat, newlon)

    @staticmethod
    def marker_position_to_angle(x, y, z):
        angle_x = math.atan2(x, z)
        angle_y = math.atan2(y, z)

        return (angle_x, angle_y)

    @staticmethod
    def camera_to_uav(x_cam, y_cam):
        x_uav = -y_cam
        y_uav = x_cam
        return (x_uav, y_uav)

    @staticmethod
    def uav_to_ne(x_uav, y_uav, yaw_rad):
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)

        north = x_uav * c - y_uav * s
        east = x_uav * s + y_uav * c
        return (north, east)

    @staticmethod
    def check_angle_descend(angle_x, angle_y, angle_desc):
        return (math.sqrt(angle_x ** 2 + angle_y ** 2) <= angle_desc)

    def localise_and_land(self, message):

        marker_found = message.isVisible
        x_cm = message.cartesianLocation.x
        y_cm = message.cartesianLocation.y
        z_cm = message.cartesianLocation.z

        if marker_found:
            x_cm, y_cm = self.camera_to_uav(x_cm, y_cm)
            uav_location = self.vehicle.location.global_relative_frame

            # -- If high altitude, use baro rather than visual
            if uav_location.alt >= 5.0:
                print
                z_cm = uav_location.alt * 100.0

            angle_x, angle_y = self.marker_position_to_angle(x_cm, y_cm, z_cm)

            if time.time() >= self.time_0 + 1.0 / self.freq_send:
                time_0 = time.time()
                # print ""
                print " "
                print "Altitude = %.0fcm" % z_cm
                print "Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f" % (
                    x_cm, y_cm, angle_x * rad_2_deg, angle_y * rad_2_deg)

                north, east = self.uav_to_ne(x_cm, y_cm, self.vehicle.attitude.yaw)
                print "Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg" % (
                    north, east, vehicle.attitude.yaw * rad_2_deg)

                marker_lat, marker_lon = get_location_metres(uav_location, north * 0.01, east * 0.01)
                # -- If angle is good, descend
                if check_angle_descend(angle_x, angle_y, angle_descend):
                    print "Low error: descending"
                    location_marker = LocationGlobalRelative(marker_lat, marker_lon,
                                                             uav_location.alt - (land_speed_cms * 0.01 / self.freq_send))
                else:
                    location_marker = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)

                vehicle.simple_goto(location_marker)
                print "UAV Location    Lat = %.7f  Lon = %.7f" % (uav_location.lat, uav_location.lon)
                print "Commanding to   Lat = %.7f  Lon = %.7f" % (location_marker.lat, location_marker.lon)

            # --- COmmand to land
            if z_cm <= land_alt_cm:
                if vehicle.mode == "GUIDED":
                    print (" -->>COMMANDING TO LAND<<")
                    vehicle.mode = "LAND"
