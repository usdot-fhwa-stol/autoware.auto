# Copyright (C) 2024 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the 'License'); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

# This file is intended to launch point_cloud_fusion_nodes on CARMA freightliner trucks
# consisting of two Velodyne LiDARs

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from ament_index_python import get_package_share_directory

import os

env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

point_cloud_fusion_node_pkg_prefix = get_package_share_directory('point_cloud_fusion_nodes')
point_cloud_fusion_node_param_file = os.path.join(point_cloud_fusion_node_pkg_prefix,
                                                  'param/point_cloud_fusion.param.yaml')


def generate_launch_description():

    point_cloud_fusion_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='point_cloud_fusion_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package='point_cloud_fusion_nodes',
                plugin='autoware::perception::filters::point_cloud_fusion_nodes::PointCloudFusionNode',
                name='point_cloud_fusion_node',
                parameters=[point_cloud_fusion_node_param_file],
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : GetLogLevel('point_cloud_fusion_nodes', env_log_levels) },
                ],
                remappings=[
                    ('output_topic', 'lidar/points_raw'),
                    ('input_topic1', 'velodyne_1/lidar/points_xyzi'),
                    ('input_topic2', 'velodyne_2/lidar/points_xyzi'),

                ]
            ),
            ComposableNode(
                package='point_type_adapter',
                plugin='autoware::tools::point_type_adapter::PointTypeAdapterNode',
                name='velodyne_1_point_type_adapter_node',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : GetLogLevel('point_type_adapter', env_log_levels) },
                ],
                remappings=[('points_raw', 'velodyne_1/lidar/points_raw'),
                            ('points_xyzi', 'velodyne_1/lidar/points_xyzi')]
            ),
            ComposableNode(
                package='point_type_adapter',
                plugin='autoware::tools::point_type_adapter::PointTypeAdapterNode',
                name='velodyne_2_point_type_adapter_node',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : GetLogLevel('point_type_adapter', env_log_levels) },
                ],
                remappings=[('points_raw', 'velodyne_2/lidar/points_raw'),
                            ('points_xyzi', 'velodyne_2/lidar/points_xyzi')],
            )
        ]
    )

    return launch.LaunchDescription([point_cloud_fusion_container])
