##launch file
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    pkg_share = get_package_share_directory('hipnuc_imu_can')
    config = os.path.join(
        pkg_share,
        'config',
        'hipnuc_config.yaml',
    )

    with open(config, 'r') as f:
        y = yaml.safe_load(f) or {}

    node_params = (y.get('canImuPublisher') or {}).get('ros__parameters') or {}
    can_if = str(node_params.get('can_port', 'can0'))
    bitrate = int(node_params.get('baud_rate', 500000))

    ip_cmd = (
        f"set -e; "
        f"if ! ip link show {can_if} >/dev/null 2>&1; then "
        f"  echo '[hipnuc_imu_can] ERROR: interface {can_if} not found. Check driver/udev/device.'; "
        f"  exit 1; "
        f"fi; "
        f"sudo -n ip link set {can_if} down || true; "
        f"sudo -n ip link set {can_if} type can bitrate {bitrate}; "
        f"sudo -n ip link set {can_if} up; "
        f"ip -details link show {can_if}"
    )

    bring_can_up = ExecuteProcess(
        cmd=['bash', '-lc', ip_cmd],
        output='screen'
    )

    talker_node = Node(
        package='hipnuc_imu_can',
        executable='talker',
        name='canImuPublisher',
        parameters=[config],
        output='screen',
    )

    listener_node = Node(
        package='hipnuc_imu_can',
        executable='listener',
        output='screen'
    )

    def on_can_setup_exit(event, context):
        rc = getattr(event, "returncode", None)
        if rc == 0:
            return [
                LogInfo(msg=f"[hipnuc_imu_can] CAN setup OK ({can_if}, {bitrate}). Starting Node..."),
                talker_node,
                listener_node,
            ]
        else:
            return [
                LogInfo(msg=f"[hipnuc_imu_can] ERROR: CAN setup failed (returncode={rc}). Shutting down, node will NOT start."),
                EmitEvent(event=Shutdown(reason="can setup failed")),
            ]
    
    start_or_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=bring_can_up,
            on_exit=on_can_setup_exit,
        )
    )

    return LaunchDescription([
        bring_can_up,
        start_or_shutdown,
    ])



