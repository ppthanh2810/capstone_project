| scenario | profile | ok/exp | succeeded | statuses | coll | min clr m | t s | ct rms m | ct max m | ω flips | stop after event s | stop dist m | compute p99/max ms | cmd period p99 ms |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| blocked_wall | custom | 5/5 | 0 | aborted | 0 | 0.099 | 20.7 | 0.000 | 0.000 | 0.0 | 0.05 | 0.000 | 2.77/5.39 | 102 |
| blocked_wall | nav2_dwb | 5/5 | 0 | aborted | 0 | 0.047 | 26.8 | 0.003 | 0.008 | 0.0 | 0.13 | 0.000 | n/a/n/a | 102 |
| blocked_wall | nav2_dwb_la | 5/5 | 0 | aborted | 0 | 0.099 | 26.9 | 0.000 | 0.001 | 0.0 | 0.10 | 0.000 | n/a/n/a | 101 |
| cancel_midway | custom | 5/5 | 0 | canceled | 0 | n/a | 4.1 | 0.000 | 0.000 | 0.0 | 0.46 | 0.046 | 3.01/3.15 | 102 |
| cancel_midway | nav2_dwb | 5/5 | 0 | canceled | 0 | n/a | 4.0 | 0.000 | 0.003 | 0.0 | 0.41 | 0.038 | n/a/n/a | 101 |
| cancel_midway | nav2_dwb_la | 5/5 | 0 | canceled | 0 | n/a | 4.1 | 0.000 | 0.002 | 0.0 | 0.46 | 0.041 | n/a/n/a | 101 |
| corridor_ok | custom | 5/5 | 5 | succeeded | 0 | 0.298 | 26.3 | 0.000 | 0.002 | 0.0 | n/a | n/a | 3.06/5.59 | 102 |
| corridor_ok | nav2_dwb | 5/5 | 5 | succeeded | 0 | 0.291 | 34.3 | 0.001 | 0.009 | 1.6 | n/a | n/a | n/a/n/a | 101 |
| corridor_ok | nav2_dwb_la | 5/5 | 5 | succeeded | 0 | 0.295 | 34.3 | 0.001 | 0.006 | 2.2 | n/a | n/a | n/a/n/a | 101 |
| goal_yaw_pi | custom | 5/5 | 5 | succeeded | 0 | n/a | 19.4 | 0.000 | 0.000 | 0.0 | n/a | n/a | 2.88/3.60 | 102 |
| goal_yaw_pi | nav2_dwb | 5/5 | 5 | succeeded | 0 | n/a | 20.7 | 0.004 | 0.009 | 0.0 | n/a | n/a | n/a/n/a | 102 |
| goal_yaw_pi | nav2_dwb_la | 5/5 | 5 | succeeded | 0 | n/a | 21.6 | 0.005 | 0.010 | 3.0 | n/a | n/a | n/a/n/a | 102 |
| l_turn | custom | 5/5 | 5 | succeeded | 0 | n/a | 24.9 | 0.027 | 0.133 | 0.0 | n/a | n/a | 3.07/3.53 | 101 |
| l_turn | nav2_dwb | 5/5 | 5 | succeeded | 0 | n/a | 27.1 | 0.020 | 0.112 | 0.0 | n/a | n/a | n/a/n/a | 101 |
| l_turn | nav2_dwb_la | 5/5 | 5 | succeeded | 0 | n/a | 29.2 | 0.012 | 0.077 | 2.2 | n/a | n/a | n/a/n/a | 102 |
| obstacle_near_path | custom | 5/5 | 5 | succeeded | 0 | 0.160 | 26.2 | 0.000 | 0.000 | 0.0 | n/a | n/a | 3.01/7.02 | 101 |
| obstacle_near_path | nav2_dwb | 5/5 | 5 | succeeded | 0 | 0.157 | 29.3 | 0.004 | 0.008 | 1.4 | n/a | n/a | n/a/n/a | 102 |
| obstacle_near_path | nav2_dwb_la | 4/5 | 4 | aborted, succeeded | 0 | 0.159 | 32.2 | 0.002 | 0.019 | 2.4 | 0.05 | 0.000 | n/a/n/a | 101 |
| overspeed_start | custom | 5/5 | 5 | succeeded | 0 | n/a | 31.3 | 0.000 | 0.000 | 0.0 | n/a | n/a | 3.08/4.11 | 102 |
| overspeed_start | nav2_dwb | 5/5 | 5 | succeeded | 0 | n/a | 33.9 | 0.005 | 0.014 | 0.0 | n/a | n/a | n/a/n/a | 101 |
| overspeed_start | nav2_dwb_la | 5/5 | 5 | succeeded | 0 | n/a | 34.4 | 0.001 | 0.006 | 0.4 | n/a | n/a | n/a/n/a | 102 |
| s_curve | custom | 5/5 | 5 | succeeded | 0 | n/a | 36.9 | 0.039 | 0.078 | 3.0 | n/a | n/a | 3.61/6.46 | 102 |
| s_curve | nav2_dwb | 5/5 | 5 | succeeded | 0 | n/a | 39.2 | 0.025 | 0.055 | 3.4 | n/a | n/a | n/a/n/a | 101 |
| s_curve | nav2_dwb_la | 5/5 | 5 | succeeded | 0 | n/a | 40.1 | 0.024 | 0.056 | 3.0 | n/a | n/a | n/a/n/a | 101 |
| stale_odom | custom | 5/5 | 0 | aborted | 0 | n/a | 4.8 | 0.000 | 0.000 | 0.0 | 0.91 | 0.113 | 3.12/3.13 | 101 |
| stale_odom | nav2_dwb | 0/5 | 5 | succeeded | 0 | n/a | 21.1 | 0.004 | 0.009 | 0.0 | 17.35 | 2.345 | n/a/n/a | 101 |
| stale_odom | nav2_dwb_la | 0/5 | 5 | succeeded | 0 | n/a | 21.7 | 0.001 | 0.005 | 0.4 | 17.98 | 2.399 | n/a/n/a | 101 |
| stale_scan | custom | 5/5 | 0 | hung (no action result, cancel not processed) | 0 | n/a | 35.0 | 0.000 | 0.000 | 0.0 | 1.51 | 0.202 | 3.11/3.11 | 101 |
| stale_scan | nav2_dwb | 5/5 | 0 | hung (no action result, cancel not processed) | 0 | n/a | 35.0 | 0.002 | 0.006 | 0.0 | 1.51 | 0.194 | n/a/n/a | 101 |
| stale_scan | nav2_dwb_la | 5/5 | 0 | hung (no action result, cancel not processed) | 0 | n/a | 35.0 | 0.001 | 0.002 | 0.0 | 1.51 | 0.191 | n/a/n/a | 101 |
| start_backwards | custom | 5/5 | 5 | succeeded | 0 | n/a | 22.3 | 0.004 | 0.015 | 0.0 | n/a | n/a | 3.19/3.85 | 102 |
| start_backwards | nav2_dwb | 5/5 | 5 | succeeded | 0 | n/a | 23.2 | 0.029 | 0.085 | 1.0 | n/a | n/a | n/a/n/a | 101 |
| start_backwards | nav2_dwb_la | 0/5 | 0 | aborted | 0 | n/a | 10.0 | 0.000 | 0.000 | 0.0 | 0.05 | 0.000 | n/a/n/a | 101 |
| straight | custom | 5/5 | 5 | succeeded | 0 | n/a | 19.6 | 0.000 | 0.000 | 0.0 | n/a | n/a | 2.33/2.52 | 101 |
| straight | nav2_dwb | 5/5 | 5 | succeeded | 0 | n/a | 21.2 | 0.002 | 0.008 | 0.0 | n/a | n/a | n/a/n/a | 101 |
| straight | nav2_dwb_la | 5/5 | 5 | succeeded | 0 | n/a | 21.8 | 0.001 | 0.007 | 2.4 | n/a | n/a | n/a/n/a | 101 |
