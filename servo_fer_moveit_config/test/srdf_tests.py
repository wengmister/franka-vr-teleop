#  Copyright (c) 2023 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from os import path

from ament_index_python.packages import get_package_share_directory
import xacro

fer_xacro_file_name = path.join(get_package_share_directory('servo_fer_moveit_config'), 'srdf',
                                'fer_arm.srdf.xacro')


def test_load():
    urdf = xacro.process_file(fer_xacro_file_name).toxml()
    assert urdf.find('fer_rightfinger') != -1


def test_load_without_gripper():
    urdf = xacro.process_file(fer_xacro_file_name,
                              mappings={'hand': 'false'}).toxml()
    assert urdf.find('fer_rightfinger') == -1


def test_load_with_arm_id():
    urdf = xacro.process_file(fer_xacro_file_name,
                              mappings={'arm_id': 'totally_different_arm'}).toxml()
    assert urdf.find('totally_different_arm_joint1') != -1


if __name__ == '__main__':
    pass
