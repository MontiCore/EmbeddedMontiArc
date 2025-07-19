/* (c) https://github.com/MontiCore/monticore */
const CONSTANTS = require("./constants");
const Process   = require("child_process");
const Log = require("log4js");

class Chrome {
	constructor() {
		this.processes = {};
		this.logger = Log.getLogger("CHROME");
		this.logger.level = "debug";
	}
	
	open(url) {
		//this.close(url);
		this.processes[url] = Process.spawn(CONSTANTS.EXECUTABLES.CHROME, [url]);
		this.logger.info("Tab has been opened: %s.", url);
	}
	
	close(url) {
		const process = this.processes[url];

		if(process) {
			process.kill();
			delete this.processes[url];
			this.logger.info("Tab has been closed: %s.", url);
        }
	}
}

module.exports = new Chrome();
