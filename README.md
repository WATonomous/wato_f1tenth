# WATonomous F1Tenth

## Windows Setup for AutoDRIVE F1Tenth Simulator

#### Development Space Setup
1. Download Ubuntu in Microsoft Store (The one without any extension)
2. Install Docker Destop: Settings -> Resources -> WSL Integration -> Enable Ubuntu
3. Go into Ubuntu and setup username and password, if first time installing
4. In your home directory (~), run: `git clone https://github.com/WATonomous/wato_f1tenth.git`
5. Go into the repo: `cd wato_f1tenth`
6. Start VSCode: `code .`
7. Make sure the `watod-config.sh` file has: `ACTIVE_MODULES="vis_tools robot sim"` and `MODE_OF_OPERATION="develop"`. 
8. Run `./watod build && ./watod up` to start the containers. Future starts can just be `./watod up` if you don't need to rebuild containers.
9. Download the Docker extension for VSCode. 
10. From the extension pannel, right click on the `-robot_dev-1` container and attach a VSCode.
11. In the opened VSCode window select the `/home/bolty/ament_ws` folder. This will be where you do your development!

#### Connecting the AutoDRIVE Simulator
1. Download the `practice` simulator from: https://autodrive-ecosystem.github.io/competitions/f1tenth-sim-racing-iros-2024/#resources
2. Unzip the download and run `AutoDRIVE Simulator.exe` inside the folder
3. Open the left menu on the simulator application
4. Press on the button "Disconnected" to attempt connection to your running watod containers. The default port should already be 4567.
5. You should see "Connected"- this means your physics simulator is connected to your watod containers and everything is setup correctly!
6. There is built-in Keyboard Teleop in the simulator. Click on "Autonomous" to set driving mode to "Manual", you will be able to drive with WASD.

#### Note for other OS (Ubuntu and MacOS)
This guide was only written & tested on Windows. 
- Try the above steps and if you have any problems, ping an F1Tenth Lead on the Discord channel!
