# WATonomous F1Tenth

## Windows Setup for AutoDRIVE F1Tenth Simulator

#### Repo Setup
1. Download Ubuntu in Microsoft Store (The one without any extension)
2. Install Docker Destop: Settings -> Resources -> WSL Integration -> Enable Ubuntu
3. Go into Ubuntu and setup username and password, if first time installing
4. In the your home directory (~), run: `git clone https://github.com/WATonomous/wato_f1tenth.git`
5. Go into the repo by running: cd wato_f1tenth
6. Attach vscode to this repo by running (Asssuming you have vscode): code . 
7. Run ./watod up to build and up the container

#### Connecting the AutoDRIVE Simulator
8. Download the `practice` simulator from: https://autodrive-ecosystem.github.io/competitions/f1tenth-sim-racing-iros-2024/#resources
9. Unzip the download and run `AutoDRIVE Simulator.exe` inside the folder
10. Open the left menu on the simulator application
11. Press on the button "Disconnected" to attempt connection to your running watod containers. The default port should already be 4567.
12. You should see "Connected"- this means your physics simulator is connected to your watod containers and everything is setup correct!
