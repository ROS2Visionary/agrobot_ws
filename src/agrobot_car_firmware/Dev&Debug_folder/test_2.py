import binascii
from struct import pack


def encode_speed(speed_mm_s):
 
    # 对速度进行限制，以免速度过快，弄坏电机
    if not -300 <= speed_mm_s <= 300:
        raise ValueError("速度必须在 -300 到 300 mm/s 范围内")

    if speed_mm_s >= 0:
        # 正数速度(前进)
        high_byte = (speed_mm_s // 256) & 0x7F  # 高字节，清除最高位
        low_byte = speed_mm_s % 256
    else:
        # 负数速度(后退)
        high_byte = (speed_mm_s >> 8) & 0xFF  # 取高8位
        low_byte = speed_mm_s & 0xFF          # 取低8位

    return high_byte, low_byte # 表示指令高字节和低字节的两个字节


def decode_speed(high_byte, low_byte):
    # 组合高字节和低字节
    combined_value = (high_byte << 8) | low_byte

    # 判断是正数还是负数
    if high_byte & 0x80:  # 如果高字节的最高位为1，表示负数
        speed_mm_s = combined_value - 0x10000
    else:
        speed_mm_s = combined_value

    return speed_mm_s


if __name__ == "__main__":

    high_byte, low_byte = encode_speed(276)
    print(f"{high_byte} - {low_byte}")
    speed = decode_speed(high_byte, low_byte)
    print(speed)

