# Health of the computer running the stack, published by system_monitor about once a second.
# NaN means not available on this machine (no GPU load, no power monitor, ...).
std_msgs/Header header
string hostname

float32 cpu_percent            # all cores
float32[] cpu_core_percent
float32 cpu_freq_mhz           # mean over cores
float32 gpu_percent
float32 ram_used_percent
float32 ram_used_mb
float32 disk_used_percent      # the filesystem holding / (images, bags)

Temperature[] temperatures
float32 max_temperature_c

PowerRail[] power_rails        # empty when the machine has no power monitor
float32 module_power_w         # Jetson module input power (VDD_IN); NaN elsewhere

string[] alarms                # anything worth a look, e.g. "VDD_IN current critical"
