// Ground braking test: launch to 4000 ERPM (~0.87 m/s), step the speed command down to 1500 ERPM, then coast.
// Shows whether the speed controller brakes (negative current) or only coasts. Logged at 50 Hz.
// Needs ~3 m of clear floor and someone ready to catch the car. VESC timeout (1 s) stops it if this dies.
// Run on the car: agents/scripts/vesc/run_vesc_script.sh brake_test
import QtQuick 2.7

Item {
    property string port: "/dev/ttyACM0"
    property real t0: 0
    property bool testActive: false
    property string phase: "connect"
    property real servoCenter: 0.4735

    function now() { return (Date.now() - t0) / 1000.0 }
    function log(msg) { console.log("TEST " + now().toFixed(3) + " " + msg) }
    function stopAll(reason) {
        if (!testActive) return
        testActive = false
        VescIf.commands().setCurrent(0)
        log("STOP " + reason)
        finish.start()
    }

    Component.onCompleted: {
        t0 = Date.now()
        if (!VescIf.connectSerial(port, 115200)) { log("CONNECT FAILED"); Qt.quit() }
    }

    Connections {
        target: VescIf
        function onFwRxChanged(rx, limited) {
            if (!rx || testActive || phase !== "connect") return
            if (limited) { log("LIMITED MODE, abort"); Qt.quit(); return }
            t0 = Date.now()
            testActive = true
            log("connected, starting")
            tick.start()
        }
    }

    Connections {
        target: VescIf.commands()
        function onValuesReceived(v, mask) {
            log("phase=" + phase + " rpm=" + v.rpm.toFixed(0) + " i_motor=" + v.current_motor.toFixed(2) +
                " i_in=" + v.current_in.toFixed(2) + " duty=" + v.duty_now.toFixed(3) + " v_in=" + v.v_in.toFixed(2) +
                " t_mos=" + v.temp_mos.toFixed(1) + " fault=" + v.fault_str)
            if (v.fault_str !== "FAULT_CODE_NONE") stopAll("fault " + v.fault_str)
            if (Math.abs(v.rpm) > 7000) stopAll("rpm over limit")
        }
    }

    Timer {
        id: tick; interval: 20; repeat: true
        onTriggered: {
            if (!testActive) { stop(); return }
            var t = now(), c = VescIf.commands()
            c.setServoPos(servoCenter)
            if (t < 1.0)       { phase = "idle";     c.setCurrent(0) }
            else if (t < 2.5)  { phase = "rpm4000";  c.setRpm(4000) }
            else if (t < 4.0)  { phase = "rpm1500";  c.setRpm(1500) }
            else if (t < 5.0)  { phase = "coast";    c.setCurrent(0) }
            else { stopAll("done"); return }
            c.getValues()
        }
    }

    Timer { id: finish; interval: 300; onTriggered: { VescIf.disconnectPort(); Qt.quit() } }
    Timer { interval: 12000; running: true; onTriggered: stopAll("hard timeout") }
}
