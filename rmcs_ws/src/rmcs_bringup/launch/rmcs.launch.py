from typing import List, Optional
import os

from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class MyLaunchDescriptionEntity(LaunchDescriptionEntity):
    def visit(
        self, context: "LaunchContext"
    ) -> Optional[List["LaunchDescriptionEntity"]]:
        entities = []

        robot_config = LaunchConfiguration("robot").perform(context)
        if robot_config.startswith("auto."):
            is_automatic = True
            robot_name = robot_config[5:]
        else:
            is_automatic = False
            robot_name = robot_config

        entities.append(
            LogInfo(
                msg=f"Starting RMCS on robot '{robot_config}'{'(automatic)' if is_automatic else ''} -> {robot_name}.yaml"
            )
        )

        config_dir = os.path.join(
            FindPackageShare("rmcs_bringup").perform(context), "config"
        )
        parameters = []
        if robot_name in {
            "wheel-leg-joints-turn-sweep",
            "wheel-leg-joints-knee-sweep",
            "wheel-leg-joints-coupled-sweep",
            "wheel-leg-joints-friction-sweep",
            "wheel-leg-joints-load-sweep",
        }:
            parameters.append(os.path.join(config_dir, "wheel-leg-joints-sweep.yaml"))
        parameters.append(os.path.join(config_dir, robot_name + ".yaml"))

        entities.append(
            Node(
                package="rmcs_executor",
                executable="rmcs_executor",
                parameters=parameters,
                respawn=True,
                respawn_delay=1.0,
                output="log",  # stdout and stderr are logged to launch log file and stderr to the screen.
            )
        )

        if is_automatic:
            pass

        return entities


def generate_launch_description():
    ld = LaunchDescription([MyLaunchDescriptionEntity()])

    return ld
