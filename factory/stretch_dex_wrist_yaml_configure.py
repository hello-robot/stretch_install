#!/usr/bin/env python

from __future__ import print_function
import stretch_body.hello_utils

dex_wrist_user_yaml={
    'robot':{'use_collision_manager':1, 'tool':'tool_stretch_dex_wrist'},
    'params':['stretch_tool_share.stretch_dex_wrist.params'],
    'tool_stretch_gripper':{'baud':115200},
    'tool_none':{'baud':115200},
    'wrist_yaw':{'baud':115200},
    'stretch_gripper':{'range_t':[0,6415],'zero_t':4017,'baud':115200},
    'lift':{'i_feedforward':0.75},
    'hello-motor-lift':{'gains':{'i_safety_feedforward':0.75}}
}
user_yaml=stretch_body.hello_utils.read_fleet_yaml('stretch_re1_user_params.yaml')
stretch_body.hello_utils.overwrite_dict(overwritee_dict=user_yaml, overwriter_dict=dex_wrist_user_yaml)
stretch_body.hello_utils.write_fleet_yaml('stretch_re1_user_params.yaml',user_yaml)