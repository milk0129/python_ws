import serial
import time

def parse_gps_data(nmea_sentence):
    parts = nmea_sentence.split(',')
    if parts[0] == '$GPRMC' and parts[2] == 'A':  # A는 유효한 위치를 의미
        latitude = parts[3]
        longitude = parts[5]
        return latitude, longitude
    return None, None

# 시리얼 포트 설정
serial_port = ('/dev/ttyUSB0')
baud_rate = 115200

# 시리얼 포트 열기
with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
    time.sleep(2)  # 시리얼 포트 안정화 대기
    
    while True:
        if ser.in_waiting > 0:
            nmea_sentence = ser.readline().decode('ascii', errors='replace').strip()
            latitude, longitude = parse_gps_data(nmea_sentence)
            
            if latitude and longitude:
                print(f"위도: {latitude}, 경도: {longitude}")
