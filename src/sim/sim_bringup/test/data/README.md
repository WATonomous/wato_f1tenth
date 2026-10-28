# Contract test data

`autodrive_contract/` (about 150 KB) is 12 s of raw AutoDRIVE output (`/autodrive/roboracer_1/*` sensors
and its `/tf`), replayed by `test/test_hal_contract.py` through `sim.launch.yaml` to check the
`/hal/*` contract. It is the one bag committed to the repo (AGENTS.md rule 10).

Re-record it only when AutoDRIVE's topics or the devkit version change. With the AutoDRIVE
simulator open on the host and the watod stack down (the bridge needs port 4567), run only the
bridge on a private ROS domain, so `/tf` holds AutoDRIVE's transforms and nothing of ours:

```bash
IMG=ghcr.io/watonomous/wato_f1tenth/sim/sim:big_infra_update
DATA=src/sim/sim_bringup/test/data
docker run -d --rm --name contract_bridge --network host --ipc host -e ROS_DOMAIN_ID=88 \
  --entrypoint bash $IMG -c 'source /opt/ros/jazzy/setup.bash && source /opt/watonomous/setup.bash && \
  ros2 run autodrive_roboracer autodrive_bridge'
rm -rf $DATA/autodrive_contract
# Connect AutoDRIVE to the bridge, then record 12 s. Both containers need --ipc host and the
# same user (root): the default middleware moves data over shared memory between processes on
# one machine, and the bridge's segments are root-owned. chown hands the files back to you.
docker run --rm --network host --ipc host -e ROS_DOMAIN_ID=88 -v "$PWD/$DATA:/out" \
  --entrypoint bash $IMG -c "source /opt/ros/jazzy/setup.bash && \
  timeout -s INT 12 ros2 bag record -s mcap --storage-preset-profile zstd_small -o /out/autodrive_contract \
  /tf /autodrive/roboracer_1/lidar /autodrive/roboracer_1/imu /autodrive/roboracer_1/odom \
  /autodrive/roboracer_1/ips /autodrive/roboracer_1/steering /autodrive/roboracer_1/throttle \
  /autodrive/roboracer_1/left_encoder /autodrive/roboracer_1/right_encoder; \
  chown -R $(id -u):$(id -g) /out/autodrive_contract"
docker stop contract_bridge
```

Then run `./watod test sim sim_bringup`.
