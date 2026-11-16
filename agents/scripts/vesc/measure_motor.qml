// Measure motor R, L (twice) and flux linkage. Nothing is written to the VESC config.
// WHEELS OFF THE GROUND: the flux linkage step spins the motor up in open loop (~30% duty).
// Run on the car: agents/scripts/vesc/run_vesc_script.sh measure_motor
import QtQuick 2.7

Item {
    property string port: "/dev/ttyACM0"
    property int step: 0      // 0 fw, 1 RL #1, 2 RL #2, 3 flux, 4 done
    property real rSum: 0
    property real lSum: 0
    property int n: 0
    property real iMeas: 20.0       // VESC Tool default: l_current_max / 3
    property real erpmPerSec: 2000  // VESC Tool default
    property real duty: 0.3         // VESC Tool default

    function log(m) { console.log("MEAS " + m) }
    function quitWith(m) { log(m); VescIf.disconnectPort(); Qt.quit() }

    Component.onCompleted: { if (!VescIf.connectSerial(port, 115200)) quitWith("CONNECT FAILED") }

    Connections {
        target: VescIf
        function onFwRxChanged(rx, limited) {
            if (!rx || step !== 0) return
            if (limited) { quitWith("LIMITED MODE, abort"); return }
            settle.start()
        }
    }
    Timer { id: settle; interval: 2000; onTriggered: {
        var mc = VescIf.mcConfig()
        log("stored: R=" + (mc.getParamDouble("foc_motor_r") * 1e3).toFixed(3) + " mOhm, L=" +
            (mc.getParamDouble("foc_motor_l") * 1e6).toFixed(3) + " uH, ld-lq=" +
            (mc.getParamDouble("foc_motor_ld_lq_diff") * 1e6).toFixed(3) + " uH, flux=" +
            (mc.getParamDouble("foc_motor_flux_linkage") * 1e3).toFixed(4) + " mWb")
        step = 1; log("measuring R & L #1 (noise, no rotation)"); VescIf.commands().measureRL()
    } }

    Connections {
        target: VescIf.commands()
        function onMotorRLReceived(r, l, ldLq) {
            if (step !== 1 && step !== 2) return
            if (r < 1e-9 && l < 1e-9) { quitWith("R & L measurement FAILED"); return }
            log("R&L #" + step + ": R=" + (r * 1e3).toFixed(3) + " mOhm, L=" + l.toFixed(3) + " uH, ld-lq=" + ldLq.toFixed(3) + " uH")
            rSum += r; lSum += l; n++
            if (step === 1) { step = 2; pause.start(); return }
            step = 3
            var r = rSum / n, l = lSum / n
            log("measuring flux linkage at " + iMeas + " A, " + erpmPerSec + " ERPM/s, duty " + duty + " (motor spins)")
            VescIf.commands().measureLinkageOpenloop(iMeas, erpmPerSec, duty, r, l / 1e6)
        }
        function onMotorLinkageReceived(flux) {
            if (step !== 3) return
            step = 4
            if (flux < 1e-9) { quitWith("flux linkage measurement FAILED"); return }
            var kv = 60.0 / (2 * Math.PI * Math.sqrt(3) * 2 * flux)
            log("flux=" + (flux * 1e3).toFixed(4) + " mWb, implies " + kv.toFixed(0) + " kV for a 4-pole motor, observer gain " +
                (1e-3 / (flux * flux)).toFixed(2) + " (x1M)")
            quitWith("DONE, nothing written")
        }
    }
    Timer { id: pause; interval: 1500; onTriggered: { log("measuring R & L #2"); VescIf.commands().measureRL() } }
    Timer { interval: 90000; running: true; onTriggered: quitWith("TIMEOUT at step " + step) }
}
