import pyrealsense2 as rs

def get_realsense_resolutions():
    """
    获取Intel RealSense相机支持的所有分辨率和帧率信息。

    返回:
        dict: 包含设备名称、序列号、传感器名称及其支持的分辨率、格式和帧率的字典。
    """
    # 创建一个context对象，这将持有和管理所有相机的资源
    context = rs.context()

    # 检查连接的设备数量
    if len(context.devices) == 0:
        print("没有找到任何设备.")
        return None

    # 获取连接的设备
    device = context.devices[0]
    device_info = {
        "name": device.get_info(rs.camera_info.name),
        "serial_number": device.get_info(rs.camera_info.serial_number),
        "sensors": []
    }

    # 获取支持的流
    for sensor in device.query_sensors():
        sensor_info = {
            "sensor_name": sensor.get_info(rs.camera_info.name),
            "stream_profiles": []
        }
        
        for stream_profile in sensor.get_stream_profiles():
            stream_info = {
                "stream_type": stream_profile.stream_type().name,
                "format_type": stream_profile.format().name,
                "width": stream_profile.as_video_stream_profile().width(),
                "height": stream_profile.as_video_stream_profile().height(),
                "fps": stream_profile.fps()
            }
            sensor_info["stream_profiles"].append(stream_info)
        
        device_info["sensors"].append(sensor_info)

    return device_info


def print_realsense_resolutions(resolutions):
    """
    输出Intel RealSense相机的支持分辨率、格式和帧率信息，并附带中文注释解释不同类型传感器的作用。

    参数:
        resolutions (dict): 通过get_realsense_resolutions()函数获取的设备分辨率信息字典。
    """
    if resolutions:
        print(f"设备名称: {resolutions['name']}")
        print(f"序列号: {resolutions['serial_number']}")

        for sensor in resolutions["sensors"]:
            print(f"\n传感器: {sensor['sensor_name']}")
            for stream in sensor["stream_profiles"]:
                print(f"类型: {stream['stream_type']}, 格式: {stream['format_type']}, 分辨率: {stream['width']}x{stream['height']}, 帧率: {stream['fps']}fps")
            
            # 根据传感器类型添加注释
            if "Stereo Module" in sensor['sensor_name']:
                print("\nStereo Module(立体模块)")
                print("  红外传感器(Infrared):")
                print("    作用:用于生成立体深度图像，通过红外光测量物体距离。")
                print("    应用:低光环境下的物体检测、3D重建等。")
                print("  深度传感器(Depth):")
                print("    作用:生成深度数据，每个像素到相机的距离。")
                print("    应用:机器人导航、3D扫描、手势识别等。")

            if "RGB Camera" in sensor['sensor_name']:
                print("\nRGB Camera(RGB 相机)")
                print("  作用:捕捉彩色图像，用于物体识别、颜色检测等。")
                print("  应用:计算机视觉、视觉导航、用户界面等。")

            if "Motion Module" in sensor['sensor_name']:
                print("\nMotion Module(运动模块)")
                print("  加速度计(Accel):")
                print("    作用:测量相机在三维空间中的加速度变化。")
                print("    应用:运动跟踪、稳定性控制、姿态估计等。")
                print("  陀螺仪(Gyro):")
                print("    作用:测量角速度，提供旋转信息。")
                print("    应用:姿态估计、图像稳定、虚拟现实等。")


if __name__ == "__main__":
    resolutions = get_realsense_resolutions()
    
    if resolutions:
        print_realsense_resolutions(resolutions)
