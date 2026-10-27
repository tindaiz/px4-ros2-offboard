from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_topics = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
        (
            "/depth_camera/points@sensor_msgs/msg/PointCloud2"
            "[gz.msgs.PointCloudPacked"
        ),
        (
            "/world/default/model/x500_depth_lidar_0/link/base_link/"
            "sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ),
        (
            "/world/default/model/x500_depth_lidar_0/link/camera_link/"
            "sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image"
        ),
        (
            "/world/default/model/x500_depth_lidar_0/link/camera_link/"
            "sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo"
            "[gz.msgs.CameraInfo"
        ),
        (
            "/world/default/model/x500_depth_lidar_0/link/camera_link/"
            "sensor/camera_imu/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ),
        (
            "/world/default/model/x500_depth_lidar_0/link/link/"
            "sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan"
            "[gz.msgs.LaserScan"
        ),
        (
            "/world/default/model/x500_depth_lidar_0/link/link/"
            "sensor/lidar_2d_v2/scan/points@sensor_msgs/msg/PointCloud2"
            "[gz.msgs.PointCloudPacked"
        ),
        (
            "/world/default/model/x500_depth_lidar_0/link/lidar_sensor_link/"
            "sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ),
        (
            "/world/default/model/x500_depth_lidar_0/link/base_link/"
            "sensor/air_pressure_sensor/air_pressure"
            "@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure"
        ),
        (
            "/world/default/model/x500_depth_lidar_0/link/base_link/"
            "sensor/magnetometer_sensor/magnetometer"
            "@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer"
        ),
        (
            "/world/default/model/x500_depth_lidar_0/link/base_link/"
            "sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix"
            "[gz.msgs.NavSat"
        ),
        (
            "/model/x500_depth_lidar_0/odometry_with_covariance"
            "@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance"
        ),
    ]

    return LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_bridge",
                arguments=bridge_topics,
                output="screen",
            )
        ]
    )
