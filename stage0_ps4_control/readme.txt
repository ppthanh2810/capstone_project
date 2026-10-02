// ============================================================
// MOTION CONTROL
// ============================================================

    linear.x angular.z

    double left = (x - wheel_distance_ / 2.0 * z) / wheel_radius_;

    double right = (x + wheel_distance_ / 2.0 * z) / wheel_radius_;

    double left_rpm = left * rpm_factor_;
    
    double right_rpm = -right * rpm_factor_;
    
// ============================================================
// MOTOR FEEDBACK
// ============================================================    
    
    auto [left_rpm_raw, right_rpm_raw] = motors_.getRpm();

    auto [left_travelled_raw, right_travelled_raw] = motors_.getWheelsTravelled();

    double left_rpm = static_cast<double>(left_rpm_raw);

    double right_rpm = -static_cast<double>(right_rpm_raw);

    double left_travelled = static_cast<double>(left_travelled_raw);

// ============================================================
// PARAMETERS
// ============================================================    
    
   static constexpr double pi_ = 3.14159265358979323846;

   static constexpr double rpm_factor_ = 30.0 / pi_;

   double wheel_radius_ = 0.0535;
   double wheel_distance_ = 0.34;
   
   
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 PS4 / Joystick
       	│
       	│ /joy
 	▼
 teleop_joy
       	│
       	│ /cmd_vel_joy
   	▼
 ┌────────────────────┐
 │     twist_mux      │◄──────── /cmd_vel_nav
 └─────────┬──────────┘           ▲
           │                      │
           │ /cmd_vel             │
       	    ▼                      │
 ┌────────────────────┐           │
 │  zlac8015d_move    │           │
 └─────────┬──────────┘           │
           │                      │
           │ Modbus/RS485         │
      	    ▼                      │
       ZLAC8015D                  Nav2 (sau này)
      motor driver

        │
        │ wheel velocity/feedback
	▼
     /wheel_feedback
        │
          	▼
 ┌────────────────────┐
 │  zlac8015d_odom    │      ← Stage 1
 └─────────┬──────────┘
        │
        │ /wheel_odom
         	▼
          EKF
