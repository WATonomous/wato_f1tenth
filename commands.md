# For Max Power
sudo nvpmodel -m 2      # switch to MAXN_SUPER
sudo jetson_clocks      # lock CPU/GPU/EMC clocks to max, kill the on-demand ramp-up 

# For particle filter launch
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1

# For MPPI specific task pinning on cores
sudo taskset -cp 0-2 $(pgrep -f particle_filter)
sudo taskset -cp 3-5 $(pgrep -f mppi_node)
sudo taskset -cp 5   $(pgrep -f opponent_predictor)

