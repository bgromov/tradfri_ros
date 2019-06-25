#!/usr/bin/env python3

import sys
import uuid
import asyncio
import copy
from functools import partial

from colormath.color_conversions import convert_color
from colormath.color_objects import sRGBColor, XYZColor

from pytradfri import Gateway
from pytradfri.api.aiocoap_api import APIFactory
from pytradfri.error import *
from pytradfri.util import load_json, save_json

import rospy
import rospkg

from std_msgs.msg import ColorRGBA

class TradfriAsyncRos:
    def __init__(self):
        self.gateway_address = rospy.get_param('~gateway_address')
        self.psk_key = rospy.get_param('~psk_key', None)

        self.lights_param = rospy.get_param('~lights', None)

        transition_time = rospy.get_param('~transition_time', 0.5) # in seconds
        self.transition_time = int(transition_time / 0.1) # in 0.1 of a second

        rospack = rospkg.RosPack()
        self.psk_config_file = rospy.get_param('~psk_config_file', rospack.get_path('tradfri_ros') + '/config/tradfri_psk.conf')

        self.conf = load_json(self.psk_config_file)
        self.connected = False
        self.color_cmd = None

    async def init_api(self):
        try:
            identity = self.conf[self.gateway_address].get('identity')
            psk = self.conf[self.gateway_address].get('key')
            api_factory = APIFactory(host=self.gateway_address, psk_id=identity, psk=psk)
        except KeyError:
            identity = uuid.uuid4().hex
            api_factory = APIFactory(host=self.gateway_address, psk_id=identity)

            try:
                psk = await api_factory.generate_psk(self.psk_key)
                # rospy.loginfo('Generated PSK: {}'.format(psk))

                self.conf[self.gateway_address] = {'identity': identity,
                                   'key': psk}
                save_json(self.psk_config_file, self.conf)
                rospy.logwarn(f'Generated PSK was stored to {self.psk_config_file}')

            except AttributeError:
                rospy.logfatal("Please provide the 'Security Code' from the "
                               "back of your Tradfri gateway using the "
                               "~psk_key param.")
                raise
            except RequestTimeout as e:
                rospy.logfatal(e.message)
                raise

            except Exception as e:
                rospy.logfatal("Unknown error occurred. Make sure to provide the "
                               "correct 'Security Code' to ~psk_key param and "
                               "gateway IP address to ~gateway_address")
                sys.exit(-1)

        self.api = api_factory.request

        # Connect to gateway and get configured devices (these are not necessarily available)
        while not self.connected and not rospy.is_shutdown():
            try:
                rospy.loginfo(f'Connecting to gateway: {self.gateway_address}')
                gateway = Gateway()

                devices_command = gateway.get_devices()
                devices_commands = await self.api(devices_command)

                rospy.loginfo('Requesting configured devices')
                devices = await self.api(devices_commands)

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
            self.lights_param = {k: {'alias': idx+1} for idx, k in enumerate(self.lights.keys())}
            rospy.set_param('~lights', self.lights_param)

        # Print all lights
        rospy.loginfo('Configured (paired) lights: {}'.format([f'ID: {k}, Name: {v.name}' for k, v in self.lights.items()]))

        self.lights_set = {int(light_id, 16) for light_id in self.lights_param.keys()}
        self.discover_set = copy.deepcopy(self.lights_set)

        # Check if user-specified devices are actually available
        while len(self.discover_set) and not rospy.is_shutdown():
            try:
                rospy.loginfo('Looking for lights: {}'.format({hex(dev_id) for dev_id in self.discover_set}))
                devices = await self.api(devices_commands)
                found_set = {dev.id for dev in devices if dev.reachable}

                self.discover_set.difference_update(found_set)

                # Wait 1 sec and try again
                if len(self.discover_set):
                    await asyncio.sleep(1.0)

            except RequestTimeout:
                rospy.logwarn('Request timed out. Keep trying...')


        # Create subscribers for configured lights
        self.subs_set_color = []
        self.pubs_color = {}
        for dev_id, params in self.lights_param.items():
            func = partial(self.set_color_cb, dev_id=dev_id)
            sub = rospy.Subscriber('light{}/set_color'.format(params['alias']), ColorRGBA, func)
            self.subs_set_color.append(sub)

        if len(self.lights_param):
            self.sub_set_all = rospy.Subscriber('all_lights/set_color', ColorRGBA, self.set_all_color_cb)

        rospy.loginfo('Ready')

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

        if div > 0:
            x = xyz.xyz_x / div * 65535
            y = xyz.xyz_y / div * 65535

            x_val = int(x)
            y_val = int(y)

            set_color_cmd = light.light_control.set_xy_color(x_val, y_val, transition_time=self.transition_time)

        dimmer_val = int(a * 254)
        set_brightness_cmd = None

        set_brightness_cmd = light.light_control.set_dimmer(dimmer_val, transition_time=self.transition_time)

        if set_color_cmd and set_brightness_cmd:
            set_color_cmd.combine_data(set_brightness_cmd)
            return set_color_cmd
        else:
            return set_brightness_cmd

    def set_color_cb(self, msg, dev_id):
        color_cmd = self.make_color_cmd(msg, dev_id)
        if color_cmd:
            # Throttle at ~6 Hz rate
            coro = self.throttle(rospy.Duration(0.17), [dev_id], self.api, color_cmd)
            if coro:
                asyncio.ensure_future(coro, loop=self.event_loop)

            ## Ignore throttling
            # asyncio.ensure_future(self.api(color_cmd), loop=self.event_loop)

    def set_all_color_cb(self, msg):
        for dev_id in self.lights.keys():
            self.set_color_cb(msg, dev_id)

    async def wait_ros(self):
        while not rospy.is_shutdown():
            await asyncio.sleep(0.1)
        print('ROS is shutting down...')

    async def run(self):
        self.event_loop = asyncio.get_event_loop()
        await asyncio.gather(self.init_api(), self.wait_ros())

if __name__ == '__main__':
    rospy.init_node('tradfri_ros')

    tradfri = TradfriAsyncRos()

    try:
        asyncio.run(tradfri.run())
    except rospy.ROSInterruptException:
        rospy.logdebug('Exiting')