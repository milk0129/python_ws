import serial
import time
import re

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
TIMEOUT = 1
filename = 'empty_map.pgm'

def parse_gps_data(gps_data):
    match = re.search(r'(\d{2})(\d{2}\.\d+),([NS]),(\d{3})(\d{2}\.\d+),([EW])', gps_data)
    if match:
        lat_deg = float(match.group(1))
        lat_min = float(match.group(2))
        lat_hem = match.group(3)
        lon_deg = float(match.group(4))
        lon_min = float(match.group(5))
        lon_hem = match.group(6)

        latitude = lat_deg + lat_min / 60
        longitude = lon_deg + lon_min / 60

        if lat_hem == 'S':
            latitude = -latitude
        if lon_hem == 'W':
            longitude = -longitude

        return latitude, longitude
    return None, None

def read_gps_data():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
        time.sleep(2)  # Sensor initialization wait

        while True:
            line = ser.readline().decode('utf-8').strip()
            print(f"Received GPS Data: {line}")

            latitude, longitude = parse_gps_data(line)
            if latitude is not None and longitude is not None:
                print(f"Parsed Latitude: {latitude}, Longitude: {longitude}")
                return latitude, longitude

def create_empty_pgm(filename, width, height):
    # 빈 PGM 파일 생성
    with open(filename, 'w') as f:
        f.write(f'P2\n')  # PGM 파일 포맷
        f.write(f'{width} {height}\n')  # 이미지 크기
        f.write('255\n')  # 최대 그레이스케일 값
        for _ in range(height):
            f.write(' '.join(['0'] * width) + '\n')  # 모든 픽셀을 0으로 초기화

def add_gps_to_pgm(filename, latitude, longitude):
    # GPS 정보를 PGM 파일에 추가
    with open(filename, 'a') as f:  # 'a' 모드로 파일을 열어 추가
        f.write(f'# GPS Latitude: {latitude}, Longitude: {longitude}\n')

# 빈 PGM 파일 생성
pgm_filename = 'output.pgm'
create_empty_pgm(pgm_filename, 100, 100)  # 100x100 크기의 빈 PGM 파일 생성

# GPS 위도와 경도 값 추가
gps_latitude = 37.5665  # 예시 위도
gps_longitude = 126.978  # 예시 경도
add_gps_to_pgm(pgm_filename, gps_latitude, gps_longitude)

print(f'{pgm_filename} 파일이 생성되었습니다.')

if __name__ == "__main__":
    latitude, longitude = read_gps_data()
    if latitude is not None and longitude is not None:
        add_gps_to_pgm(filename, latitude, longitude)
