// Ground current sweep: launches from standstill at fixed current (no speed PID), logged at 50 Hz.
// Steps: idle 1 s, then for 8 A, 15 A, 25 A: drive 0.8 s, coast 1.5 s.
// A launch that passes 5000 ERPM coasts early; the test aborts on any fault or |rpm| > 7000.
// Needs ~3 m of clear floor and someone ready to catch the car. VESC timeout (1 s) stops it if this dies.
// Run on the car: agents/scripts/vesc/run_vesc_script.sh current_sweep
import QtQuick 2.7

Item {
    property string port: "/dev/ttyACM0"
    property real t0: 0
    property bool testActive: false
    property string phase: "connect"
    property real servoCenter: 0.476
    property var steps: [8, 15, 25]
    property real driveS: 0.8
    property real coastS: 1.5
    property var cut: ({})   // launches that passed 5000 ERPM

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
            if (phase.indexOf("cur") === 0 && v.rpm > 5000) cut[phase] = true
        }
    }

    Timer {
        id: tick; interval: 20; repeat: true
        onTriggered: {
            if (!testActive) { stop(); return }
            var t = now(), c = VescIf.commands()
            c.setServoPos(servoCenter)
            if (t < 1.0) { phase = "idle"; c.setCurrent(0); c.getValues(); return }
            var k = Math.floor((t - 1.0) / (driveS + coastS)), r = (t - 1.0) - k * (driveS + coastS)
            if (k >= steps.length) { stopAll("done"); return }
            if (r < driveS) {
                phase = "cur" + steps[k]
                if (cut[phase]) c.setCurrent(0); else c.setCurrent(steps[k])
            } else {
                phase = "coast" + steps[k]
                c.setCurrent(0)
            }
            c.getValues()
        }
    }

    Timer { id: finish; interval: 300; onTriggered: { VescIf.disconnectPort(); Qt.quit() } }
    Timer { interval: 15000; running: true; onTriggered: stopAll("hard timeout") }
}
