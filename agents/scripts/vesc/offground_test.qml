// Off-ground check: read state, spin 3000 then 6000 ERPM, coast, small steering sweep.
// WHEELS OFF THE GROUND. Aborts on any fault, |rpm| > 9000, or the hard timeout.
// Run on the car: agents/scripts/vesc/run_vesc_script.sh offground_test
import QtQuick 2.7

Item {
    property string port: "/dev/ttyACM0"
    property real t0: 0
    property bool testActive: false
    property string phase: "connect"
    property real servoCenter: 0.4735   // steering_angle_to_servo_offset in vesc.yaml

    function now() { return (Date.now() - t0) / 1000.0 }
    function log(msg) { console.log("TEST " + now().toFixed(2) + " " + msg) }

    function stopAll(reason) {
        testActive = false
        VescIf.commands().setCurrent(0)
        VescIf.commands().setServoPos(servoCenter)
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
            log("connected, starting")
            t0 = Date.now()
            testActive = true
            tick.start()
        }
    }

    Connections {
        target: VescIf.commands()
        function onValuesReceived(v, mask) {
            log("phase=" + phase + " rpm=" + v.rpm.toFixed(0) + " i_motor=" + v.current_motor.toFixed(2) +
                " i_in=" + v.current_in.toFixed(2) + " duty=" + v.duty_now.toFixed(3) + " v_in=" + v.v_in.toFixed(2) +
                " t_mos=" + v.temp_mos.toFixed(1) + " t_motor=" + v.temp_motor.toFixed(1) + " fault=" + v.fault_str)
            if (testActive && v.fault_str !== "FAULT_CODE_NONE") stopAll("fault " + v.fault_str)
            if (testActive && Math.abs(v.rpm) > 9000) stopAll("rpm over limit")
        }
    }

    // 20 Hz: send the command for the current phase (keeps the VESC timeout fed) and poll values
    Timer {
        id: tick; interval: 50; repeat: true
        onTriggered: {
            if (!testActive) { stop(); return }
            var t = now(), c = VescIf.commands()
            if (t < 1.0)       { phase = "idle";      c.setCurrent(0) }
            else if (t < 3.0)  { phase = "rpm3000";   c.setRpm(3000) }
            else if (t < 5.0)  { phase = "rpm6000";   c.setRpm(6000) }
            else if (t < 7.0)  { phase = "coast";     c.setCurrent(0) }
            else if (t < 7.6)  { phase = "servo_c";   c.setCurrent(0); c.setServoPos(servoCenter) }
            else if (t < 8.2)  { phase = "servo_l";   c.setCurrent(0); c.setServoPos(servoCenter - 0.08) }
            else if (t < 8.8)  { phase = "servo_r";   c.setCurrent(0); c.setServoPos(servoCenter + 0.08) }
            else if (t < 9.4)  { phase = "servo_c";   c.setCurrent(0); c.setServoPos(servoCenter) }
            else { stopAll("done"); return }
            c.getValues()
        }
    }

    Timer { id: finish; interval: 300; onTriggered: { VescIf.disconnectPort(); Qt.quit() } }
    Timer { interval: 20000; running: true; onTriggered: stopAll("hard timeout") }
}
