// Read-only: is the VESC's IMU working? Asks the firmware which IMU chip it initialized (terminal
// command imu_type_internal), reads the app config's IMU type and rate, then reads 20 IMU samples.
// Nothing is written and the motor is never commanded.
// Run on the car: agents/scripts/vesc/run_vesc_script.sh imu_check
import QtQuick 2.7

Item {
    property string port: "/dev/ttyACM0"
    property int step: 0          // 0 fw, 1 appconf, 2 terminal, 3 samples, 4 done
    property int samples: 0
    property var acc: [0, 0, 0]
    property var gyr: [0, 0, 0]

    function log(m) { console.log("IMU " + m) }
    function quitWith(m) { log(m); VescIf.disconnectPort(); Qt.quit() }

    Component.onCompleted: {
        if (!VescIf.connectSerial(port, 115200)) quitWith("CONNECT FAILED")
    }

    Connections {
        target: VescIf
        function onFwRxChanged(rx, limited) {
            if (!rx || step !== 0) return
            if (limited) { quitWith("LIMITED MODE, abort"); return }
            settle.start()
        }
    }
    Timer { id: settle; interval: 2000; onTriggered: { step = 1; VescIf.commands().getAppConf() } }

    Connections {
        target: VescIf.appConfig()
        function onUpdated() {
            if (step !== 1) return
            var a = VescIf.appConfig()
            log("appconf imu_conf.type = " + a.getParamEnum("imu_conf.type") + " (0 off, 1 internal)" +
                ", sample_rate_hz = " + a.getParamInt("imu_conf.sample_rate_hz") +
                ", mode = " + a.getParamEnum("imu_conf.mode"))
            step = 2
            VescIf.commands().sendTerminalCmd("imu_type_internal")
            termWait.start()
        }
    }

    Connections {
        target: VescIf.commands()
        function onPrintReceived(str) {
            if (step === 2) log("terminal: " + str.trim())
        }
        function onValuesImuReceived(v, mask) {
            if (step !== 3) return
            samples++
            if (samples <= 5 || samples === 20) {
                log("sample " + samples + " mask " + mask +
                    " acc(g) " + v.accX.toFixed(3) + " " + v.accY.toFixed(3) + " " + v.accZ.toFixed(3) +
                    " gyro(deg/s) " + v.gyroX.toFixed(2) + " " + v.gyroY.toFixed(2) + " " + v.gyroZ.toFixed(2) +
                    " rpy(rad) " + v.roll.toFixed(3) + " " + v.pitch.toFixed(3) + " " + v.yaw.toFixed(3))
            }
            acc = [Math.max(acc[0], Math.abs(v.accX)), Math.max(acc[1], Math.abs(v.accY)), Math.max(acc[2], Math.abs(v.accZ))]
            gyr = [Math.max(gyr[0], Math.abs(v.gyroX)), Math.max(gyr[1], Math.abs(v.gyroY)), Math.max(gyr[2], Math.abs(v.gyroZ))]
            if (samples >= 20) {
                step = 4
                var g = Math.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2])
                log("max |acc| per axis (g): " + acc.map(function(x) { return x.toFixed(3) }).join(" ") +
                    "  |max gyro| (deg/s): " + gyr.map(function(x) { return x.toFixed(2) }).join(" "))
                quitWith(g > 0.5 ? "IMU WORKS: gravity is visible (~1 g expected)" : "IMU READS ZERO: no gravity in the accelerometer")
            }
        }
    }

    Timer { id: termWait; interval: 1500; onTriggered: { step = 3; poll.start() } }
    Timer { id: poll; interval: 100; repeat: true; onTriggered: { if (step === 3) VescIf.commands().getImuData(0xFFFF); else stop() } }
    Timer { interval: 25000; running: true; onTriggered: quitWith("TIMEOUT at step " + step + " (" + samples + " samples)") }
}
