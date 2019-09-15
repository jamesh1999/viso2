
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros import get_default_launch_description
import launch_ros.actions


def generate_launch_description():
    ld = LaunchDescription()

    ns = "/image"

    image_proc = launch_ros.actions.Node(
        package='image_proc', node_executable='image_proc', output='screen', parameters=[{"camera_namespace": ns}])

    mono_odometer = launch_ros.actions.Node(
        package='viso2_ros', node_executable='mono_odometer', output='screen', node_namespace=ns,
        remappings=[(f"{ns}/image", f"{ns}/image_color_rect")],
        parameters=[{
            "odom_frame_id": "odom",
            "base_link_frame_id": "camera",
            "camera_height": 1.00, 
            "camera_pitch": 0.00}]
    )

    ld.add_action(image_proc)
    ld.add_action(mono_odometer)

    return ld


def main(argv=sys.argv[1:]):
    """Main."""
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    #ls = LaunchService(debug=True)
    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description())
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
