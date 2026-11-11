from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from advrf_ros2_motors_controller.robot_config import load_robot

def setup(context):

    def value(name):
        return LaunchConfiguration(name).perform(context)

    namespace  = '/' + value('namespace').strip('/')
    transport  = value('ecat_namespace') or namespace.rstrip('/') + '/ecat'
    robot      = load_robot(value('config_dir'), namespace, value('robot_description_file'), 'impedance')

    controller        = robot['nodes']['joint_command_controller_node']
    configured_joints = robot['limits']['joints']
    requested         = [name.strip() for name in value('gravity_joints').split(',') if name.strip()]
    joints            = requested or configured_joints
    if len(joints) != len(set(joints)) or any(name not in configured_joints for name in joints):
        raise ValueError('gravity_joints must be unique joints from robot_config.yaml')

    indices     = [configured_joints.index(name) for name in joints]
    motor_ids   = [controller['motor_ids'][index] for index in indices]
    joint_gains = [value for index in indices for value in controller['joint_gains'][5 * index:5 * (index + 1)]]

    gravity = Node(
        package='advrf_ros2_gravity_compensation',
        executable='advrf_ros2_gravity_compensation',
        name='gravity_compensation',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot['description'],
            'joints': joints,
            'motor_ids': motor_ids,
            'joint_gains': joint_gains,
            'gain_ramp_duration': 3.0,
        }],
        remappings=[
            ('rx/joint_states', transport.rstrip('/') + '/rx/joint_states'),
            ('tx/motors', transport.rstrip('/') + '/tx/motors'),
        ],
    )
    return [gravity]


def generate_launch_description():
    defaults = {
        'config_dir':             '',
        'namespace':              '/advrf/robot',
        'ecat_namespace':         '',
        'robot_description_file': 'auto',
        'gravity_joints':         '',
    }
    descriptions = {
        'config_dir':             'Robot directory containing config/ and urdf/',
        'namespace':              'Robot namespace',
        'ecat_namespace':         'EtherCAT namespace; defaults to <namespace>/ecat',
        'robot_description_file': 'URDF/Xacro path',
        'gravity_joints':         'Comma-separated joints controlled by gravity compensation',
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=default, description=descriptions[name])
        for name, default in defaults.items()
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=setup)])
