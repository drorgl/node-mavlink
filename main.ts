
import SerialPort = require("serialport");
import { mavlink, IMessage } from "./src/mavlink";

// Open serial port
let port = new SerialPort("COM24", {
	baudRate: 115200
});

// When port is open, start up mavlink
port.on("open", () => {
	console.log("Serial Port is ready");

	// listening for system 1 component 1
	let m = new mavlink(1, 1);

	// When mavlink is ready, assign some listeners
	m.on("ready", () => {
		console.log("Mavlink is ready!");

		// Parse any new incoming data
		port.on("data", (data: Buffer) => {
			m.parse(data);
		});

		m.on("message", (message_name: string, message: IMessage, fields: any) => {
			console.log("message", message_name, /* message, */fields);
		});

		// Attitude listener
		m.on("ATTITUDE", (message: IMessage, fields: any) => {
			// Do something interesting with Attitude data here
			// console.log("ATTITUDE Roll is ", fields.roll, "Pitch is ", fields.pitch);
		});

		// Create a few messages and print them to screen
		m.createMessage("ATTITUDE", {
			"time_boot_ms": 30,
			"roll": 0.1,
			"pitch": 0.2,
			"yaw": 0.3,
			"rollspeed": 0.4,
			"pitchspeed": 0.5,
			"yawspeed": 0.6
		}, echoMessage);

		m.createMessage("PARAM_VALUE", {
			"param_id": "MY_PI",
			"param_value": 3.14159,
			"param_type": 5,
			"param_count": 100,
			"param_index": 55
		}, echoMessage);

		m.createMessage("GPS_STATUS", {
			"satellites_visible": 5,
			"satellite_prn": [1, 2, 3, 4, 5],
			"satellite_used": [2, 3, 4, 5, 6],
			"satellite_elevation": [3, 4, 5, 6, 7],
			"satellite_azimuth": [4, 5, 6, 7, 8],
			"satellite_snr": [5, 6, 7, 8, 9]
		}, echoMessage);
	});
});

function echoMessage(message: IMessage) {
	console.log(message);
}
