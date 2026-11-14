// OFF-GROUND high-speed check: steps 10k, 20k, 30k, 40k, 46.2k ERPM (~2.2 to 10 m/s at the wheels), brake to 5k, coast.
// CAR ON A STAND, WHEELS CLEAR. Aborts on any fault, |rpm| > 52000, MOSFET > 70 C or v_in < 10.5 V.
// Run on the car: agents/scripts/vesc/run_vesc_script.sh highspeed_offground
import QtQuick 2.7

Item {
    property string port: "/dev/ttyACM0"
    property real t0: 0
    property bool testActive: false
    property string phase: "connect"
    property real servoCenter: 0.4735
    //            [end time s, command, value]
    property var plan: [[1.0, "cur", 0], [2.5, "rpm", 10000], [4.0, "rpm", 20000], [5.5, "rpm", 30000],
                        [7.0, "rpm", 40000], [9.0, "rpm", 46200], [11.0, "rpm", 5000], [13.0, "cur", 0]]

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
            if (Math.abs(v.rpm) > 52000) stopAll("rpm over limit")
            if (v.temp_mos > 70) stopAll("mosfet hot")
            if (v.v_in < 10.5) stopAll("battery low")
        }
    }

    Timer {
        id: tick; interval: 20; repeat: true
        onTriggered: {
            if (!testActive) { stop(); return }
            var t = now(), c = VescIf.commands(), step = null
            for (var i = 0; i < plan.length; i++) { if (t < plan[i][0]) { step = plan[i]; break } }
            if (step === null) { stopAll("done"); return }
            c.setServoPos(servoCenter)
            phase = step[1] + step[2]
            if (step[1] === "rpm") c.setRpm(step[2]); else c.setCurrent(step[2])
            c.getValues()
        }
    }

    Timer { id: finish; interval: 300; onTriggered: { VescIf.disconnectPort(); Qt.quit() } }
    Timer { interval: 20000; running: true; onTriggered: stopAll("hard timeout") }
}
