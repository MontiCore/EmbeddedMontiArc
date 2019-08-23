/* (c) https://github.com/MontiCore/monticore */
const ChildProcess = require("child_process");
const {PATHS} = require("./constants");
const Log = require("log4js");

class Process {
	constructor() {
		this.logger = Log.getLogger("PROCESS");
		this.logger.level = "debug";
	}

	spawn(file, args, options) {
		options = options || {};
		options.env = { JAVA_HOME: PATHS.JDK, NUMBER_OF_PROCESSORS: 1, PROGRAMFILES: "C:\Program Files" };

		const process = ChildProcess.spawn(file, args, options);

		process.stdout.on("data", this.onStdOut.bind(this));
		process.stderr.on("data", this.onStdErr.bind(this));

		return process;
	}

	onStdOut(buffer) {
		const data = buffer.toString();

		this.logger.info(data);
	}

	onStdErr(buffer) {
		const data = buffer.toString();

		this.logger.error(data);
	}
}

module.exports = new Process();
