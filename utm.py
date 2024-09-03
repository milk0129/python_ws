"""def create_empty_pgm_with_utm(filename, width, height, max_gray, utm_zone, utm_easting, utm_northing):
    with open(filename, 'w') as f:
        # PGM 헤더 작성
        f.write("P2\n")  # ASCII 포맷으로 PGM 파일을 생성
        f.write(f"# UTM Zone: {utm_zone}\n")
        f.write(f"# UTM Easting: {utm_easting}\n")
        f.write(f"# UTM Northing: {utm_northing}\n")
        f.write(f"{width} {height}\n")
        f.write(f"{max_gray}\n")
        
        # 빈 픽셀 데이터 추가 (모든 값을 0으로 설정)
        for _ in range(height):
            f.write(" ".join(["255"] * width) + "\n")

# 예시 사용
create_empty_pgm_with_utm(
    filename="empty_utm.pgm",
    width=100,
    height=100,
    max_gray=255,
    utm_zone=52,  # UTM Zone 예시
    utm_easting=500000,  # UTM Easting 예시
    utm_northing=4649776  # UTM Northing 예시
) """

def create_empty_pgm(filename, width=100, height=100):
    with open(filename, 'w') as f:
        f.write(f'P2\n{width} {height}\n255\n')
        for _ in range(height):
            f.write(' '.join(['255']*width) + '\n') #255 = white

create_empty_pgm('empty_map.pgm')
