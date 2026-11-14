// Ground launch test: two launches from standstill, logged at 50 Hz.
// Needs about 5 m of clear straight floor; someone ready to catch the car.
// Steps: idle 1 s, 2500 ERPM (~0.54 m/s) 1.5 s, coast 2 s, 4500 ERPM (~0.97 m/s) 1.5 s, coast 2 s.
// Aborts on any fault, |rpm| > 7000, or the hard timeout. VESC timeout (1 s) stops it if this dies.
// Run on the car: agents/scripts/vesc/run_vesc_script.sh ground_test
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
            else if (t < 2.5)  { phase = "rpm2500";  c.setRpm(2500) }
            else if (t < 4.5)  { phase = "coast1";   c.setCurrent(0) }
            else if (t < 6.0)  { phase = "rpm4500";  c.setRpm(4500) }
            else if (t < 8.0)  { phase = "coast2";   c.setCurrent(0) }
            else { stopAll("done"); return }
            c.getValues()
        }
    }

    Timer { id: finish; interval: 300; onTriggered: { VescIf.disconnectPort(); Qt.quit() } }
    Timer { interval: 15000; running: true; onTriggered: stopAll("hard timeout") }
}
