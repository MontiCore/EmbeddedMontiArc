/* (c) https://github.com/MontiCore/monticore */
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


function onOCLZipped() {
	const Server = App.listen(PORTS.SHARED, onServerStarted);

	Server.timeout = 30 * 60 * 1000;
}

function onClusteringZipped() {
	Archiver.zip("oclverification", onOCLZipped);

}

function onAutopilotZipped() {
	Archiver.zip("clustering", onClusteringZipped);
}

function onNFPZipped() {
	Archiver.zip("autopilot", onAutopilotZipped);
}

function onSuperMarioZipped() {
	Archiver.zip("nfpverification", onNFPZipped);
}

function onPacManZipped() {
	Archiver.zip("supermario", onSuperMarioZipped);
}

function onPumpZipped() {
	Archiver.zip("pacman", onPacManZipped);
}

Archiver.zip("pump", onPumpZipped);

