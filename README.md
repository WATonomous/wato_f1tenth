# WATonomous F1Tenth

## Initial Windows Setup for visualizing in Foxglove
1. Download Ubuntu in Microsoft Store (The one without any extension)
2. Install Docker Destop: Settings -> Resources -> WSL Integration -> Enable Ubuntu
3. Go into Ubuntu and setup username and password, if first time installing
4. In the your home directory (~), run: git clone https://github.com/WATonomous/wato_f1tenth.git
5. Go into the repo by running: cd wato_f1tenth
6. Attach vscode to this repo by running (Asssuming you have vscode): code . 
7. In VS Code, manually forward the port by going to PORTS -> Add Port -> Enter 20000 for port number
8. Run ./watod up to build and up the container
9. Go to foxglove: Open Connection -> Foxglove Websocket -> Enter ws://localhost:20000
10. Now go to the Panel section on the left -> click the 3D Panel in the middle -> Topics -> Hover your cursor on the map and click the eye button to make map visible

Now you should be able to see the car with transforms and the map
