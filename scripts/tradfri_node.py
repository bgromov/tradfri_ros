#!/usr/bin/env python3

import sys

import uuid
import argparse
import threading
import time
import copy
from functools import partial

from colormath.color_conversions import convert_color
from colormath.color_objects import sRGBColor, XYZColor

from pytradfri import Gateway
from pytradfri.api.libcoap_api import APIFactory
from pytradfri.error import *
from pytradfri.util import load_json, save_json

import rospy
import rospkg

from std_msgs.msg import ColorRGBA

class TradfriRos:
    def __init__(self):
        self.publish_rate = rospy.get_param('~publish_rate', 20.0)

        self.gateway = rospy.get_param('~gateway')
        self.key = rospy.get_param('~key', None)

        self.lights_param = rospy.get_param('~lights', None)

        transition_time = rospy.get_param('~transition_time', 0.0) # in seconds
        self.transition_time = int(transition_time / 0.1) # in 0.1 of a second

        rospack = rospkg.RosPack()
        self.psk_config_file = rospy.get_param('~psk_config_file', rospack.get_path('tradfri_ros') + '/config/tradfri_psk.conf')

        self.conf = load_json(self.psk_config_file)

        self.connected = False

        self.color_cmd = None

        try:
            identity = self.conf[self.gateway].get('identity')
            psk = self.conf[self.gateway].get('key')
            api_factory = APIFactory(host=self.gateway, psk_id=identity, psk=psk)
        except KeyError:
            identity = uuid.uuid4().hex
            api_factory = APIFactory(host=self.gateway, psk_id=identity)

            try:
                psk = api_factory.generate_psk(self.key)
                rospy.loginfo('Generated PSK: {}'.format(psk))

                self.conf[self.gateway] = {'identity': identity,
                                   'key': psk}
                save_json(self.psk_config_file, self.conf)
            except AttributeError:
                raise rospy.logfatal("Please provide the 'Security Code' from the "
                                     "back of your Tradfri gateway using the "
                                     "~key param.")

        self.api = api_factory.request

        # Connect to gateway and get configured devices (these are not necessarily available)
        while not self.connected and not rospy.is_shutdown():
            try:
                rospy.loginfo('Connecting to gateway: {}'.format(self.gateway))
                gateway = Gateway()

                devices_command = gateway.get_devices()
                devices_commands = self.api(devices_command)

                rospy.loginfo('Requesting configured devices')
                devices = self.api(devices_commands)

                self.connected = True
            except RequestTimeout:
                rospy.logwarn('Request timed out. Keep trying...')

        # User canceled request, e.g. with Ctrl-C
        if not self.connected:
            return

        # Lights devices
        self.lights = {hex(dev.id): dev for dev in devices if dev.has_light_control}

        if not self.lights_param:
            # if no lights configured, set the param with default values
            self.lights_param = {k: {'alias': idx+1, 'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0}} for idx, k in enumerate(self.lights.keys())}
            rospy.set_param('~lights', self.lights_param)

        # Print all lights
        rospy.loginfo('Configured (paired) lights: {}'.format(["ID: {}, Name: {}".format(k, v.name) for k, v in self.lights.items()]))

        self.lights_set = {int(light_id, 16) for light_id in self.lights_param.keys()}
        self.discover_set = copy.deepcopy(self.lights_set)

        # Check if user-specified devices are actually available
        while len(self.discover_set) and not rospy.is_shutdown():
            try:
                rospy.loginfo('Looking for lights: {}'.format({hex(dev_id) for dev_id in self.discover_set}))
                devices = self.api(devices_commands)
                found_set = {dev.id for dev in devices if dev.reachable}

                self.discover_set.difference_update(found_set)

                # Wait 1 sec and try again
                if len(self.discover_set):
                    time.sleep(1)

            except RequestTimeout:
                rospy.logwarn('Request timed out. Keep trying...')


        # Create subscribers for configured lights
        self.subs_set_color = []
        self.pubs_color = {}
        for dev_id, params in self.lights_param.items():
            func = partial(self.set_color_cb, dev_id=dev_id)
            sub = rospy.Subscriber('light{}/set_color'.format(params['alias']), ColorRGBA, func)
            self.subs_set_color.append(sub)

            # pub = rospy.Publisher('light{}/color'.format(params['alias']), ColorRGBA, queue_size=10)
            # self.pubs_color[dev_id] = pub

            self.observe(self.api, self.lights[dev_id])

        if len(self.lights_param):
            self.sub_set_all = rospy.Subscriber('all_lights/set_color', ColorRGBA, self.set_all_color_cb)

        rospy.loginfo('Ready')

    def observe(self, api, device):
        def callback(updated_device):
            light = updated_device.light_control.lights[0]
            # dev_id = hex(light.device.id)
            # x, y = light.xy_color[0], light.xy_color[1]
            # r, g, b, a = self.xyb_to_rgba(x, y, light.dimmer)

            # self.pubs_color[dev_id].publish(ColorRGBA(r, g, b, a))
            rospy.logdebug("Received message for: %s" % light)
            pass

        def err_callback(err):
            # rospy.logerr(err)
            self.api(device.observe(callback, err_callback, duration=5.0))
            # pass

        def worker():
            self.api(device.observe(callback, err_callback, duration=5.0))

        threading.Thread(target=worker, daemon=True).start()
        rospy.loginfo('Sleeping to start observation task')
        time.sleep(0.5)

    def xyb_to_rgba(self, x, y, dimmer):
        _x = x / 65535
        _y = y / 65535
        _z = (1.0 - _x - _y)
        # _z = (dimmer / 254 * 65535)

        rgb = convert_color(XYZColor(_x, _y, _z), sRGBColor, observer='2', target_illuminant='d65')
        r, g, b = (rgb.rgb_r, rgb.rgb_g, rgb.rgb_b)
        a = dimmer / 254

        # rospy.loginfo('r: {}, g: {}, b: {}, a: {}'.format(r, g, b, a))

        return (r, g, b, a)

    def clamp(self, val, lower=0.0, upper=1.0):
        if val < lower:
            return lower

        if val > upper:
            return upper

        return val

    def throttle(self, dur, hashables, func, *args, **kwargs):
        if not hasattr(self, 'throttle_cache'):
            self.throttle_cache = {}

        now = rospy.Time.now()
        h = hash(func)
        if hashables and isinstance(hashables, list):
            for val in hashables:
                h ^= hash(val)

        if h in self.throttle_cache:
            last_time = self.throttle_cache[h]
            if now - last_time < dur:
                return None

        self.throttle_cache[h] = now

        return func(*args, **kwargs)

    def make_color_cmd(self, msg, dev_id):
        light = self.lights[dev_id]

        r, g, b, a = self.clamp(msg.r), self.clamp(msg.g), self.clamp(msg.b), self.clamp(msg.a)

        # Convert RGB to XYZ using a D65 illuminant.
        xyz = convert_color(sRGBColor(r, g, b), XYZColor, observer='2', target_illuminant='d65')

        # Convert XYZ to XY
        set_color_cmd = None
        div = (xyz.xyz_x + xyz.xyz_y + xyz.xyz_z)
        # rospy.logwarn('Light {}'.format(light.light_control.lights[0]))
        if div > 0:
            x = xyz.xyz_x / div * 65535
            y = xyz.xyz_y / div * 65535

            x_val = int(x)
            y_val = int(y)

            if (x_val, y_val) != light.light_control.lights[0].xy_color:
                rospy.logwarn('Color; new: {}, old: {}'.format((x_val, y_val), light.light_control.lights[0].xy_color))
                set_color_cmd = light.light_control.set_xy_color(x_val, y_val, transition_time=self.transition_time)

            # set_color_cmd = light.light_control.set_xy_color(x_val, y_val, transition_time=self.transition_time)

        dimmer_val = int(a * 254)
        set_brightness_cmd = None

        if dimmer_val != light.light_control.lights[0].dimmer:
            rospy.logwarn('Dimmer; new: {}, old: {}'.format(dimmer_val, light.light_control.lights[0].dimmer))
            set_brightness_cmd = light.light_control.set_dimmer(dimmer_val, transition_time=self.transition_time)

        # set_brightness_cmd = light.light_control.set_dimmer(dimmer_val, transition_time=self.transition_time)

        # # If we changed the values, let's observe
        # if set_color_cmd or set_brightness_cmd:
        #     self.observe(self.api, self.lights[dev_id])
        if set_color_cmd and set_brightness_cmd:
            set_color_cmd.combine_data(set_brightness_cmd)
            return set_color_cmd
        else:
            return set_brightness_cmd

    def set_color_cb(self, msg, dev_id):
        color_cmd = self.make_color_cmd(msg, dev_id)
        if color_cmd:
        #     # Throttle at 10 Hz rate
            # self.throttle(rospy.Duration(0.4), [dev_id], self.api, color_cmd, timeout=0.1)
            self.api(color_cmd, timeout=0.1)

    def set_all_color_cb(self, msg):
        for dev_id in self.lights.keys():
            self.set_color_cb(msg, dev_id)

        ##### FIXME: Combining commands for different lights doesn't work
        # lights = list(self.lights.keys())
        # cmd = self.make_color_cmd(msg, lights[0])
        # rospy.loginfo(cmd._data)

        # for dev_id in lights[1:]:
        #     cmd2 = self.make_color_cmd(msg, dev_id)
        #     rospy.loginfo(cmd2._data)
        #     cmd.combine_data(cmd2)

        # rospy.loginfo(cmd._data)

        # self.api(cmd)

    def run(self):
        loop_rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            # if self.color_cmd:
            #     try:
            #         self.api(self.color_cmd, timeout=0.1)
            #     except RequestTimeout:
            #         rospy.logwarn('Request timeout')

            loop_rate.sleep()

if __name__ == '__main__':
    print("Deprecated. Please use tradfri_async_node.py instead")
    sys.exit(-1)

    rospy.init_node('tradfri_ros')

    tradfri = TradfriRos()

    try:
        tradfri.run()
    except rospy.ROSInterruptException:
        rospy.logdebug('Exiting')