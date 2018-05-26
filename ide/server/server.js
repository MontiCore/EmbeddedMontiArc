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

function onPacManZipped() {
	const Server = App.listen(PORTS.SHARED, onServerStarted);

	Server.timeout = 30 * 60 * 1000;
}

function onClusteringZipped() {
	Archiver.zip("pacman", onPacManZipped);
}

function onAutopilotZipped() {
	Archiver.zip("clustering", onClusteringZipped);
}


Archiver.zip("autopilot", onAutopilotZipped);