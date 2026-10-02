Joint / frame	X	Y	Z
base_link so 
với 
base_footprint	0	0	113.5
imu_link	110	0	120.8
camera_link	170	0	162.5
lidar_link	150	0	233.65
bánh trái	0	+170	-60
bánh phải	0	-170	-60


base_footprint
      │
      └── base_link
            │
            ├── imu_link
            │
            ├── camera_link
            │      ├── camera_color_optical_frame
            │      └── camera_depth_optical_frame
            │
            └── lidar_link
                   └── laser_frame


Chi tiết	Kích thước chính	Vị trí theo SCAD 		Ghi chú
					/ mặt đất (mm)	
Đế dưới	Ø400 × 5		tâm (0, 0, 116)			đáy chính

Đế trên	Ø400 × 5		tâm (0, 0, 221)			mặt trên Z=223.5

4 trụ đỡ	Ø12 × 100		(±120, ±100, 168.5)		nối 2 tầng

Plate điện tử	170 × 63 × 10		tâm (110, 0, 228.5)		quay Z = 90°

IMU		21 × 12 × 1.6		tâm (110, 0, 234.3)		quay Z = 90°

Giá camera STL	23 × 23 × 40		đáy (170,0,223.5), 		camera_mount_photo.stl
					tâm Z≈243.5	

RealSense D435	25 × 90 × 25		frame (170, 0, 276)		STL thực tế 26.575 × 90 × 25.19

2 trụ LiDAR	Ø12 × 100		(170, ±60, 273.5)		đỡ plate LiDAR

Plate LiDAR	90 × 170 × 3		bbox tâm (155,0,325)		X từ 110→200

RPLIDAR		55.6 × 55.6 × 41.8	frame (150,0,347.15)		STL thực tế
			
Motor 
trái/phải	110 × 25 × 40		(0, ±100, 93.5)			motor block

Plate motor	50 × 10 × 80		(0, ±117.5, 73.5)		lỗ trục ở Z=53.5

Bánh chủ động	lốp Ø107 × 41.5	tâm (0, ±170, 53.5)		track = 340 mm
			
		
Toàn STL bánh	107 × 88.7 × 107	cùng tâm trục trên		88.7 gồm hub + trục
				
Caster 
trước/sau	bánh Ø75 × 30		tâm bánh (±68, 0, 37.5)		gá caster tại X=±120
