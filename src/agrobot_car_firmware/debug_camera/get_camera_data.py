import pyrealsense2 as rs
import numpy as np
import cv2

def configure_pipeline(enable_stereo=True, enable_rgb=True):
    """
    配置RealSense流,根据需求启用立体模块和RGB相机。
    """
    pipeline = rs.pipeline()
    config = rs.config()

    if enable_stereo:
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)  # 设置为最大分辨率
    if enable_rgb:
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # 设置为最大分辨率

    return pipeline, config

def get_stereo_module_data(pipeline):
    """
    获取立体模块的深度数据。
    """
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        print("未能获取深度数据")
        return None

    depth_image = np.asanyarray(depth_frame.get_data())
    return depth_image

def get_rgb_camera_data(pipeline):
    """
    获取RGB相机的彩色图像数据。
    """
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        print("未能获取彩色图像数据")
        return None

    color_image = np.asanyarray(color_frame.get_data())
    return color_image

def get_motion_module_data(sensor):
    """
    获取运动模块的加速度计和陀螺仪数据。
    """
    motion_data = {}
    frames = sensor.get_motion_frame()
    if frames:
        accel_data = frames.as_motion_frame().get_motion_data()
        gyro_data = frames.as_motion_frame().get_motion_data()

        motion_data['accel'] = (accel_data.x, accel_data.y, accel_data.z)
        motion_data['gyro'] = (gyro_data.x, gyro_data.y, gyro_data.z)
    else:
        print("未能获取运动数据")

    return motion_data

def main(enable_stereo=True, enable_rgb=True, enable_motion=True):
    # 配置RealSense流
    pipeline, config = configure_pipeline(enable_stereo, enable_rgb)
    
    # 启动流
    pipeline.start(config)

    # 获取运动模块
    ctx = rs.context()
    devices = ctx.query_devices()
    motion_sensor = None
    if enable_motion and len(devices) > 0:
        device = devices[0]
        for sensor in device.query_sensors():
            if sensor.is_motion_sensor():
                motion_sensor = sensor
                break

    try:
        while True:
            # 获取并显示Stereo Module的数据
            if enable_stereo:
                depth_image = get_stereo_module_data(pipeline)
                if depth_image is not None:
                    cv2.imshow('Depth Image', depth_image)

            # 获取并显示RGB Camera的数据
            if enable_rgb:
                color_image = get_rgb_camera_data(pipeline)
                if color_image is not None:
                    cv2.imshow('Color Image', color_image)

            # 获取并显示Motion Module的数据
            if enable_motion and motion_sensor:
                motion_data = get_motion_module_data(motion_sensor)
                if motion_data:
                    print(f"加速度计数据: {motion_data['accel']}")
                    print(f"陀螺仪数据: {motion_data['gyro']}")

            # 按键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # 启动程序，启用立体模块、RGB相机和运动模块
    main(enable_stereo=True, enable_rgb=True, enable_motion=True)
