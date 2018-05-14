const App = require("./app");
const Archiver = require("./archiver");
const Log = require("log4js");
const Chrome = require("./chrome");
const {URLS, PORTS} = require("./constants");

const Logger = Log.getLogger("APPLICATION");


Logger.level = "debug";


function onServerStarted(error) {
	if(error) Logger.error("An error occurred while starting the server: %j.", error);
	else Chrome.open(URLS.SHARED);
}

function onClusteringZipped() {
	const Server = App.listen(PORTS.SHARED, onServerStarted);

	Server.timeout = 30 * 60 * 1000;
}

function onAutopilotZipped() {
	Archiver.zip("clustering", onClusteringZipped);
}

function onPumpZipped() {
	Archiver.zip("autopilot", onAutopilotZipped);
}

Archiver.zip("pump", onPumpZipped);