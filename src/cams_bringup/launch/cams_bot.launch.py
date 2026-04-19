from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()

    # ── Mode argument ─────────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='manual',
        description='Operation mode: manual, autonomous',
    )
    ld.add_action(mode_arg)

    # Prevent pygame/SDL from needing a display
    ld.add_action(SetEnvironmentVariable('SDL_VIDEODRIVER', 'dummy'))

    # ── Manual mode nodes ─────────────────────────────────────────────────────

    # Joystick node — reads Xbox controller via pygame, publishes to /joy
    joystick_node = Node(
        package='cams_bot',
        executable='joystick_node',
        name='joystick_node',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'manual'"])
        ),
    )
    ld.add_action(joystick_node)

    # Control node — subscribes to /joy, sends D/T/B/K/R commands to Pico over UART
    control_node = Node(
        package='cams_bot',
        executable='control_node',
        name='control_node',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'manual'"])
        ),
    )
    ld.add_action(control_node)

    # ── Autonomous mode nodes (placeholder for vision tracking) ───────────────

    # Person tracking node — partner's OAK-D Pro pipeline, publishes target pose
    # tracking_node = Node(
    #     package='cams_bot',
    #     executable='tracking_node',
    #     name='tracking_node',
    #     output='screen',
    #     condition=IfCondition(
    #         PythonExpression(["'", LaunchConfiguration('mode'), "' == 'autonomous'"])
    #     ),
    # )
    # ld.add_action(tracking_node)

    # Follower node — converts target pose to drive/turn commands for Pico
    # follower_node = Node(
    #     package='cams_bot',
    #     executable='follower_node',
    #     name='follower_node',
    #     output='screen',
    #     condition=IfCondition(
    #         PythonExpression(["'", LaunchConfiguration('mode'), "' == 'autonomous'"])
    #     ),
    # )
    # ld.add_action(follower_node)

    return ld
