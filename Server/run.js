/*
    Futuwear-V2 UDP server
*/

const options = {
    httpPort      : 8080,
    webSocketPort : 8081,
    udpPort       : 8082,
}

const WebSocket = require("ws");
const http = require('http');
const finalhandler = require('finalhandler');
const serveStatic = require('serve-static');
const dgram = require('dgram');

//const bones = ["L_Arm_Inner", "L_Arm_Outer", "R_Arm_Inner", "R_Arm_Outer", "Back_Upper"];
const bones = [
    {bone_name: "L_Arm_Inner", X: 0, Y: 0, Z: 0},
    {bone_name: "L_Arm_Outer", X: 0, Y: 0, Z: 0},
    {bone_name: "R_Arm_Inner", X: 0, Y: 0, Z: 0},
    {bone_name: "R_Arm_Outer", X: 0, Y: 0, Z: 0},
    {bone_name: "Back_Upper" , X: 0, Y: 0, Z: 0},
];

// Setup HTTP server
const serve = serveStatic("./");
const httpServer = http.createServer((request, response) => {
    var done = finalhandler(request, response);
    serve(request, response, done);
}).listen(options.httpPort);

// Setup WebSocket server
const wss = new WebSocket.Server({ port: options.webSocketPort });
wss.on("connection", (client) => {
    console.log("Client connected");

    client.on("error", (err) => {
        console.log("WebSocket server error: %s", err.message);
    })

    function message_handler(message)
    {
        console.log(message);
        client.removeListener("message", message_handler);
        if (message == "browser") {
            console.log("Browser connected");
        } else {
            console.log("Unknown device tried to connect");
            console.log("Received: \"%s\"", message);
        }
    }

    client.on("message", message_handler);
});

// Setup UDP server
const server = dgram.createSocket('udp4');
server.on('error', (err) => {
    console.log(`server error:\n${err.stack}`);
    server.close();
});

var counter = 0;
server.on('message', (message, rinfo) => {
    counter++;
    try {
        var buf = new Buffer.from(message);
        for (var i = 0; i < 5; i++) {
            var bone = bones[i];

            // Process bytes received from the shirt into angles in degrees
            var value = buf.readUInt32BE(0 + i * 4, 3 + i * 4);
            var pitch = (value / (1625 * 1625)) * 360 / 1625 - 180;
            var yaw   = ((value / 1625) % 1625) * 360 / 1625 - 180;
            var roll  =  (value % 1625)         * 360 / 1625 - 180;

            // Flip X axis, and make sure that the result value is between -180 and 180
            roll = roll + 180;
            if (roll > 180)
                roll = roll - 360;

            // Reduce rotation of the upper arm from the rotation of the lower arm,
            // since the IMU values are relative to the starting point of the IMU,
            // whereas the model uses rotation relative to the parent bone
            if (i == 1 || i == 3) {
                roll  = roll  - bones[i - 1].X;
                pitch = pitch - bones[i - 1].Y;
            }
            /*
            // Reduce rotation of the back from the arms
            if (i < 4) {
                roll  = roll  - bones[4].X;
                pitch = pitch - bones[4].Y;
            }
            */

            // If the values are unchanged, don't broadcast bone
            if (bone.X == roll && bone.Y == pitch)
                continue;

            bone.X = roll;
            bone.Y = pitch;

            var broadcastMessage = JSON.stringify(bone);

            // Broadcast message to all connected clients
            wss.clients.forEach((client) => {
                client.send(broadcastMessage);
            });
        }
    }
    catch (err) {
        console.log(err.message);
    }
});

server.on('listening', () => {
    const address = server.address();
    console.log(`server listening ${address.address}:${address.port}`);
});

server.bind(options.udpPort);

// Report received message rate every 10 seconds
function logMessageRate()
{
    console.log("Receiving %d messages per second", counter / 10);
    counter = 0;
    setTimeout(logMessageRate, 10000);
}
setTimeout(logMessageRate, 10000);
