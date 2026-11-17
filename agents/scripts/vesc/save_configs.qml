// Read-only: connect to the VESC, read the motor and app config, save both as XML, quit.
// Writes ~/vesc_configs/latest/{mcconf,appconf}.xml; run_vesc_script.sh copies them to backups/<time>/.
// Nothing is ever sent to the VESC except "get config" requests.
import QtQuick 2.7

Item {
    property string port: "/dev/ttyACM0"     // VESC Tool rejects the /dev/sensors/vesc symlink
    property string dir: "/home/wato/vesc_configs/latest"
    property int step: 0   // 0 waiting for fw, 1 waiting mcconf, 2 waiting appconf, 3 done

    function quitWith(msg) {
        console.log("SAVE_CONFIGS: " + msg)
        VescIf.disconnectPort()
        Qt.quit()
    }

    Component.onCompleted: {
        console.log("SAVE_CONFIGS: connecting to " + port)
        if (!VescIf.connectSerial(port, 115200)) {
            quitWith("CONNECT FAILED")
        }
    }

    Connections {
        target: VescIf
        function onFwRxChanged(rx, limited) {
            if (!rx || step !== 0) return
            if (limited) { quitWith("LIMITED MODE (firmware/tool mismatch), not saving"); return }
            settle.start()
        }
    }

    // let VESC Tool finish its own reads on connect before asking again
    Timer {
        id: settle; interval: 2000
        onTriggered: { step = 1; VescIf.commands().getMcconf() }
    }

    Connections {
        target: VescIf.mcConfig()
        function onUpdated() {
            if (step !== 1) return
            var f = dir + "/mcconf.xml"
            console.log("SAVE_CONFIGS: mcconf saved " + VescIf.mcConfig().saveXml(f, "MCConfiguration") + " " + f)
            step = 2
            VescIf.commands().getAppConf()
        }
    }

    Connections {
        target: VescIf.appConfig()
        function onUpdated() {
            if (step !== 2) return
            var f = dir + "/appconf.xml"
            console.log("SAVE_CONFIGS: appconf saved " + VescIf.appConfig().saveXml(f, "APPConfiguration") + " " + f)
            step = 3
            quitWith("DONE")
        }
    }

    Timer { interval: 25000; running: true; onTriggered: quitWith("TIMEOUT at step " + step) }
}
