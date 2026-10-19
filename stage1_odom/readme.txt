
  
  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    zlac8015d_odom
  
    if (!wheel_initialized_) {
      last_left_travelled_ = left_travelled;
      last_right_travelled_ = right_travelled;

      wheel_initialized_ = true;

      publishOdom(current_time);

      return;
    }

    double left_distance = left_travelled - last_left_travelled_;

    double right_distance = right_travelled - last_right_travelled_;
    
    // ==========================================================
    // DIFFERENTIAL DRIVE ODOMETRY
    // ==========================================================
    
    double distance = (left_distance + right_distance) / 2.0;

    double delta_yaw = (right_distance - left_distance) / wheel_distance_;

    double middle_yaw = yaw_ + delta_yaw / 2.0;
    
    x_ += distance * std::cos(middle_yaw);
    y_ += distance * std::sin(middle_yaw);

    yaw_ += delta_yaw;
    
    
  // ============================================================
  // VELOCITY
  // ============================================================
  
  void updateVelocity(double left_rpm, double right_rpm)
  {
    double left_velocity = left_rpm * wheel_radius_ / rpm_factor_;

    double right_velocity = right_rpm * wheel_radius_ / rpm_factor_;

    double linear_velocity = (left_velocity + right_velocity) / 2.0;

    double angular_velocity = (right_velocity - left_velocity) / wheel_distance_;

    linear_velocity_ = filter_alpha_ * linear_velocity + (1.0 - filter_alpha_) * linear_velocity_;

    angular_velocity_ = filter_alpha_ * angular_velocity + (1.0 - filter_alpha_) * angular_velocity_;

    if (std::abs(linear_velocity_) < linear_deadband_) {
      linear_velocity_ = 0.0;
    }

    if (std::abs(angular_velocity_) < angular_deadband_) {
      angular_velocity_ = 0.0;
    }
  }
  
  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  imu_odom
  
                           BNO055
                      NDOF 9 trục
                           │
                           │
                   I2C: SDA, SCL
                    địa chỉ 0x29
                           │
                           │
                    ESP32-C3 Mini
                đọc và đóng gói dữ liệu
                           │
                           │
                    UART1 115200
                     TX20, RX21
                           │
                           │
                     USB-UART
                    /dev/ttyUSB1
                           │
                           │
                     imu_odom node
                           │
          ┌────────────────┼────────────────┬────────────────┐
          │                │                │                │
     	   ▼       	    ▼   		      ▼		       ▼
     /imu/data         /imu/mag        /imu_odom       /wheel_odom
                                           │                │
                                           └───────┬────────┘
                                                   │
                                                  		    ▼
                                         robot_localization EKF
                                                   │
                                                 		    ▼
                                        /odometry/filtered
                                      TF odom → base_footprint
  
  
  
  ESP32 gửi mỗi dòng gồm 15 trường: IMU,time,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz
  
  imu_odom Node thực hiện:
	- Kiểm tra dòng bắt đầu bằng IMU.
	- Kiểm tra đủ 15 trường.
	- Kiểm tra quaternion hợp lệ.
	- Chuẩn hóa quaternion.
	- Chuyển gyro từ độ/s sang rad/s.
	- Chuyển magnetometer từ µT sang tesla.
	- Tính yaw từ quaternion.
	- Xuất ba topic.
 Launch	

	Encoder bánh xe                         BNO055 + ESP32
	      │                                      │
	      │                                      │
	         ▼ 		                               ▼
	zlac8015d_odom                           imu_odom
	  /wheel_odom                            /imu_odom
	      │                                      │
	      │                                      │
	   vx, vy                              yaw, angular_z
	      │                                      │
	      └──────────────────┐        ┌───────────┘
		                 │        │
		                	 ▼	   ▼
		              ekf_filter_node
		                     │
		        ┌────────────┴────────────┐
		        │                         │
		       	▼ 		              ▼
	       /odometry/filtered                TF
		                          odom → base_footprint
  
  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  ekf
  
               Encoder bánh xe
                   │
                	    ▼
              /wheel_odom
              vx, vy
                   │
                   │
                	    ▼
                ┌─────┐
                │ EKF │
                └─────┘
             	    ▲
                   │
             yaw, yaw_rate
                   │
               /imu_odom
           	    ▲
                   │
                  IMU
  
  odom0_config:
  [false, false, false,
   false, false, false,
   true,  true,  false,
   false, false, false,
   false, false, false]
   
             0      1      2
             x      y      z

             3      4      5
           roll   pitch    yaw

             6      7      8
            vx     vy      vz

             9      10     11
          vroll  vpitch   vyaw

             12     13     14
             ax     ay     az
  

