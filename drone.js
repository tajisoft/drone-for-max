const Max = require('max-api');
const mavlink = require('mavlink');
const net = require('net');
const dgram = require('dgram');
const _ = require("underscore");

// vehicle state
var mav = null;
var mav_src = {};
var vehicle = {};
var sock = null;

// vehicle api
var api = {
    heartbeat: (cb) => {
        mav.createMessage(0, {
            "type": 2,
            "autopilot": 1,
            "base_mode": 0,
            "custom_mode": 0,
            "system_status": 4,
            "mavlink_version": "v1.0"
        }, function(msg) {
            const delay_ms = !vehicle.last_heartbeat_ms ? 1000 : (new Date() - vehicle.last_heartbeat_ms);
            _.delay(() => {
                write_out(msg.buffer);
                if (cb) {
                    cb();
                }
            }, delay_ms);
            vehicle.last_heartbeat_ms = new Date();
        });
    },
    request_data_stream: () => {
        // request data stream
        mav.createMessage(11, {
            "target_system": 1,
            "target_component": 1,
            "req_stream_id": 0,
            "req_message_rate": 1,
            "start_stop": 1
        }, function(msg) {
            console.log("request stream");
            write_out(msg.buffer);
        });
    },
    set_mode: (mode) => {
        console.log("set_mode " + mode);
        mav.createMessage(11, {
            "target_system": 1,
            "base_mode": vehicle["HEARTBEAT"]["base_mode"] || 89,
            "custom_mode": mode,
        }, (msg) => {
            write_out(msg.buffer);
        });
    },
};

// logger
function _log(msg) {
    Max.post(msg);
}

function _outlet(dat) {
    Max.outlet(dat);
}

// データ書き出し
function write_out(buf) {
    if (mav_src["type"] === "TCP") {
        sock.write(buf);
    } else if (mav_src["type"] === "UDP") {
        sock.send(buf, mav_src["port"], mav_src["host"], (err, bytes) => {
            if (err) {
                write_error(err);
                sock.close();
            }
            console.log(bytes);
            console.log("sent");
        });
    }
}

// error notify
function write_error(error) {
    Max.outlet(error);
}

// connect callback
function connect_callback() {
    console.log("tcp connected");
    mav_start();
}

// mav start
function mav_start() {
    if (mav_src["type"] === "TCP") {
        sock.on("data", (data) => {
            mav.parse(data);
        });
    } else if (mav_src["type"] === "UDP") {
        sock.on("message", (data, info) => {
            mav.parse(data);
        });
        sock.on("error", function(err) {
            console.log("client error: \n" + err.stack);
            sock.close();
            return;
        });
    }

    mav.on("COMMAND_ACK", (message, fields) => {
        console.log("COMMAND_ACK");
        console.log(fields);
    });

    mav.on("STATUSTEXT", (message, fields) => {
        if (!vehicle["STATUSTEXT"]) {
            vehicle["STATUSTEXT"] = fields;
        } else {
            _.extend(vehicle["STATUSTEXT"], fields);
        }
        // Max.outlet(vehicle);
    });

    mav.on("SERVO_OUTPUT_RAW", (message, fields) => {
        if (!vehicle["SERVO_OUTPUT_RAW"]) {
            vehicle["SERVO_OUTPUT_RAW"] = fields;
        } else {
            _.extend(vehicle["SERVO_OUTPUT_RAW"], fields);
        }
        // Max.outlet(vehicle);
    });

    mav.on("RC_CHANNELS_RAW", (message, fields) => {
        if (!vehicle["RC_CHANNELS_RAW"]) {
            vehicle["RC_CHANNELS_RAW"] = fields;
        } else {
            _.extend(vehicle["RC_CHANNELS_RAW"], fields);
        }
        // Max.outlet(vehicle);
    });

    mav.on("LOCAL_POSITION_NED", (message, fields) => {
        if (!vehicle["LOCAL_POSITION_NED"]) {
            vehicle["LOCAL_POSITION_NED"] = fields;
        } else {
            _.extend(vehicle["LOCAL_POSITION_NED"], fields);
        }
        // Max.outlet(vehicle);
    });

    mav.on("HEARTBEAT", (message, fields) => {
        if (!vehicle["is_connected"]) {
            vehicle["is_connected"] = true;
            Max.outletBang("bang connected");
        }
        if (!vehicle["HEARTBEAT"]) {
            vehicle["HEARTBEAT"] = fields;
        } else {
            _.extend(vehicle["HEARTBEAT"], fields);
        }
        // Max.outlet(vehicle);
    });

    mav.on("SYS_STATUS", (message, fields) => {
        if (!vehicle["SYS_STATUS"]) {
            vehicle["SYS_STATUS"] = fields;
        } else {
            _.extend(vehicle["SYS_STATUS"], fields);
        }
        // Max.outlet(vehicle);
    });

    mav.on("RAW_IMU", (message, fields) => {
        if (!vehicle["RAW_IMU"]) {
            vehicle["RAW_IMU"] = fields;
        } else {
            _.extend(vehicle["RAW_IMU"], fields);
        }
        // Max.outlet(vehicle);
    });

    mav.on('ATTITUDE', function(message, fields) {
        if (!vehicle["ATTITUDE"]) {
            vehicle["ATTITUDE"] = fields;
        } else {
            _.extend(vehicle["ATTITUDE"], fields);
        }
        // _outlet(vehicle);
    });

    mav.on("GLOBAL_POSITION_INT", function(message, fields) {
        if (!vehicle["GLOBAL_POSITION_INT"]) {
            vehicle["GLOBAL_POSITION_INT"] = fields;
        } else {
            _.extend(vehicle["GLOBAL_POSITION_INT"], fields);
        }
    });
}

// handle bang
Max.addHandler("bang", () => {
    Max.outlet(vehicle);
});

// connect
// type: UDP|TCP TCP is default
// host: 127.0.0.1 is default
// port: 14550 is default
Max.addHandler("connect", (t, h, p) => {
    if (vehicle["is_connected"]) {
        sock.close();
        mav.close();
        vehicle["is_connected"] = false;
        return;
    }
    mav_src = {
        "type": t || "TCP",
        "host": h || "127.0.0.1",
        "port": p || 5760
    };
    console.log(mav_src);
    mav = new mavlink(1, 1);
    sock = dgram.createSocket("udp4");
    sock.bind(mav_src["port"]);
    mav.on("ready", () => {
        if (mav_src["type"] === "UDP") {
            sock = dgram.createSocket({"type": "udp4", "reuseAddr": true});
            sock.bind(mav_src["port"] + 1);
            Max.post("Start connect: %s %s %d" % mav_src);
            mav_start()
        } else if (mav_src["type"] === "TCP") {
            console.log("tcp");
            sock = net.Socket().connect(mav_src["port"], mav_src["host"], connect_callback);
        }
    });
    mav.on("checksumFail", function(a, b, c, d, e) {
        Max.post("error checksum");
        console.log(a);
        console.log(b);
        console.log(c);
        console.log(d);
    });
});

Max.addHandler("mode", (mode) => {
    if (!vehicle["is_connected"]) {
        Max.post("error not connected");
        return;
    }
    if (mode === undefined) {
        Max.outletBang("bang " + vehicle["HEARTBEAT"]["custom_mode"]);
        return;
    }
    const res = api.set_mode(parseInt(mode));
    Max.outletBang("bang mode " + res);
});
