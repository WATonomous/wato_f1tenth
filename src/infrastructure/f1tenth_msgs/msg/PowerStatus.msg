# Power drawn from the drive pack, published by system_monitor on the car (10 Hz).
#
# The pack feeds two branches: the VESC (drive) and the power board (Jetson, lidar). The VESC
# measures its own branch; the Jetson measures its module input. Everything else on the power board
# (lidar, other 12 V / 5 V loads, converter losses) is estimated from configured values.
std_msgs/Header header

float32 pack_voltage_v         # measured by the VESC
float32 drive_current_a        # VESC input current
float32 drive_power_w
float32 compute_power_w        # Jetson module input (INA3221 VDD_IN)
float32 total_power_w          # estimate: drive + (compute + other loads) / converter efficiency
float32 total_current_a        # estimate at the pack: total_power_w / pack_voltage_v
