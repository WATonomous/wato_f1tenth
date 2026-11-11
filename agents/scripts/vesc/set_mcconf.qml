// Change motor config values listed in ~/vesc_configs/changes.json, write them, verify by read-back.
// changes.json (see changes.example.json):
//   {"label": "...", "dry_run": true,
//    "changes": [{"name": "s_pid_kd", "type": "double", "expect": 0.001, "value": 0.0002}]}
// Aborts WITHOUT WRITING if any current value differs from "expect". Saves before/after snapshots in
// ~/vesc_configs/history/. "dry_run": true reads, checks and snapshots, and writes nothing.
import QtQuick 2.7

Item {
    property string port: "/dev/ttyACM0"
    property string dir: "/home/wato/vesc_configs"
    property var job: null
    property int step: 0   // 0 fw, 1 read, 2 verify, 3 done
    property string stamp: ""

    function log(m) { console.log("SET " + m) }
    function quitWith(m) { log(m); VescIf.disconnectPort(); Qt.quit() }
    function getVal(c) {
        var mc = VescIf.mcConfig()
        return c.type === "bool" ? mc.getParamBool(c.name) : mc.getParamDouble(c.name)
    }
    function same(a, b, c) {
        if (c.type === "bool") return a === b
        return Math.abs(a - b) <= Math.max(1e-9, Math.abs(b) * 1e-4)
    }

    Component.onCompleted: {
        var d = new Date()
        stamp = d.getFullYear() + "-" + ("0" + (d.getMonth() + 1)).slice(-2) + "-" + ("0" + d.getDate()).slice(-2) +
                "_" + ("0" + d.getHours()).slice(-2) + ("0" + d.getMinutes()).slice(-2) + ("0" + d.getSeconds()).slice(-2)
        var x = new XMLHttpRequest()
        x.open("GET", "file://" + dir + "/changes.json", false)
        x.send()
        try { job = JSON.parse(x.responseText) } catch (e) { quitWith("BAD changes.json: " + e); return }
        log("job " + job.label + (job.dry_run ? " (DRY RUN)" : "") + ", " + job.changes.length + " change(s)")
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
    Timer { id: settle; interval: 2000; onTriggered: { step = 1; VescIf.commands().getMcconf() } }

    Connections {
        target: VescIf.mcConfig()
        function onUpdated() {
            if (step !== 1) return
            var mc = VescIf.mcConfig(), ok = true, i, c, cur
            for (i = 0; i < job.changes.length; i++) {
                c = job.changes[i]
                if (!mc.hasParam(c.name)) { log("UNKNOWN PARAM " + c.name); ok = false; continue }
                cur = getVal(c)
                log(c.name + ": now " + cur + ", expect " + c.expect + ", new " + c.value)
                if (!same(cur, c.expect, c)) { log("MISMATCH " + c.name); ok = false }
            }
            if (!ok) { quitWith("ABORT, nothing written"); return }
            var before = dir + "/history/" + stamp + "_before_" + job.label + ".xml"
            log("snapshot " + mc.saveXml(before, "MCConfiguration") + " " + before)
            if (job.dry_run) { quitWith("DRY RUN OK, nothing written"); return }
            for (i = 0; i < job.changes.length; i++) {
                c = job.changes[i]
                if (c.type === "bool") mc.updateParamBool(c.name, c.value)
                else mc.updateParamDouble(c.name, c.value)
            }
            step = 2
            log("writing")
            VescIf.commands().setMcconf(true)
        }
    }

    Connections {
        target: VescIf.commands()
        function onMcConfigCheckResult(notSet) {
            if (step !== 2) return
            step = 3
            var mc = VescIf.mcConfig(), bad = 0
            for (var i = 0; i < job.changes.length; i++) {
                var c = job.changes[i], v = getVal(c)
                var good = same(v, c.value, c)
                if (!good) bad++
                log("readback " + c.name + " = " + v + (good ? " OK" : " NOT SET"))
            }
            if (notSet.length > 0) log("VESC did not take: " + notSet.join(", "))
            var after = dir + "/history/" + stamp + "_after_" + job.label + ".xml"
            log("snapshot " + mc.saveXml(after, "MCConfiguration") + " " + after)
            quitWith(bad === 0 && notSet.length === 0 ? "WRITE OK" : "WRITE INCOMPLETE")
        }
    }

    Timer { interval: 25000; running: true; onTriggered: quitWith("TIMEOUT at step " + step) }
}
