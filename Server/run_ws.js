/*
    Futuwear-V2 WebSocket server
*/

const options = {
    httpPort      : 8080,
    webSocketPort : 8081,
}

const WebSocket = require("ws");

const http = require('http');
const finalhandler = require('finalhandler');
const serveStatic = require('serve-static');

var shirtClient;

//const bones = ["L_Arm_Inner", "L_Arm_Outer", "R_Arm_Inner", "R_Arm_Outer", "Back_Upper"];
const bones = ["L_Arm_Inner", "L_Arm_Outer", "R_Arm_Inner", "R_Arm_Outer", "N/A"];

const serve = serveStatic("./");
const server = http.createServer((request, response) => {
    var done = finalhandler(request, response);
    serve(request, response, done);
}).listen(options.httpPort);

function shirt_message_handler(message)
{
    try {
        // Ping back in case shirt lost its connection and is reconnecting
        if (message == "shirt") {
            shirtClient.send("ping");
            return
        }
        var buf = new Buffer.from(message, "base64");
        for (var i = 0; i < 5; i++) {
            var value = buf.readUInt32BE(0 + i * 4, 3 + i * 4);
            var pitch = (value / (1625 * 1625)) * 360 / 1625 - 180;
            var yaw   = ((value / 1625) % 1625) * 360 / 1625 - 180;
            var roll  =  (value % 1625)         * 360 / 1625 - 180;

            roll = roll + 180;
            if (roll > 180)
                roll = roll - 360;
    
            var broadcastMessage = JSON.stringify({
                "bone_name": bones[i],
                "X": roll,
                "Y": pitch,
                "Z": 0,
            });

            console.log(broadcastMessage);

            wss.clients.forEach((client) => {
                if (client != shirtClient)
                    client.send(broadcastMessage);
            });
        }
    }
    catch (err) {
        console.log(err.message);
    }
}


const wss = new WebSocket.Server({ port: options.webSocketPort });
wss.on("connection", (client) => {
    console.log("Client connected");

    client.on("error", (err) => {
        console.log(err.message);
    })

    function message_handler(message)
    {
        console.log(message);
        client.removeListener("message", message_handler);
        if (message == "browser") {
            console.log("Browser connected");
        } else if (message == "shirt") {
            console.log("Shirt connected");
            shirtClient = client;
            client.send("ping");
            client.on("message", shirt_message_handler);
        } else {
            console.log("Unknown device tried to connect");
            console.log("Received: \"%s\"", message);
        }
    }

    client.on("message", message_handler);
});