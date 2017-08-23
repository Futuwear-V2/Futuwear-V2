// Sends serial port data to the server through a WebSocket

const WebSocket = require("ws");
const SerialPort = require("serialport");
const Readline = SerialPort.parsers.Readline;

const wsc = new WebSocket("wss://protopaja2.research.comnet.aalto.fi/ws")

var port = new SerialPort("COM1", {
  baudRate: 115200
});
const parser = port.pipe(new Readline({ delimiter: '\r\n' }));

wsc.on("open", () => {
    wsc.send("shirt");
    parser.on("data", (data) => {
        var str = Buffer.from(data, "base64");
        str = String(str);
            wsc.send(data);
        console.log(String(data));
    });
});