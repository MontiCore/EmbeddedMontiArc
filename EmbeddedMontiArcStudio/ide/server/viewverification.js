/* (c) https://github.com/MontiCore/monticore */
const {BATCHES, PATHS} = require("./constants");
const Log = require("log4js");
const Path = require("path");
const Process = require("./process");

class AutoPilotVerification {
	constructor() {
        this.process = null;
        this.project = "autopilot";
        this.batch = null;
		this.logger = Log.getLogger("AUTOPILOT VIEWVERIFICATION");
        this.logger.level = "debug"
    }

    executeAll(callback) {
        const onExit = () => {
            this.logger.info("...Verification terminated.");
            callback();
        };

        this.kill();
        this.logger.info("Running Verification...");
        this.process = Process.spawn(BATCHES.AUTOPILOT.VIEWVERIFICATION.ALL, [], {
            cwd: Path.resolve(PATHS.SCRIPTS, this.project)
        });
        this.process.on("exit", onExit);
        return this.process;
    }

	execute(streamName, callback) {
        const onReadFile = (error, content) => {
            const collection = "<pre>" + content.toString().trim() + "</pre>";

            this.logger.info("...Results fetched.");
            callback(collection);
        };

        const onExit = () => {
            this.logger.info("...Verification executed.");
			callback();
        };

        this.kill();
        this.logger.info("Running Verification for %s...", streamName);
        this.process = Process.spawn(BATCHES.AUTOPILOT.VIEWVERIFICATION.SINGLE, [streamName], {
            cwd: Path.resolve(PATHS.SCRIPTS, this.project)
        });
        this.process.on("exit", onExit);
        return this.process;
    }
	
    kill() {
        if(this.process) this.process.kill();
    }
}
/*
class AutoPilotVerification extends AbstractVerification {
	constructor() {
		super("autopilot", BATCHES.AUTOPILOT.VIEWVERIFICATION);
		this.logger = Log.getLogger("AUTOPILOT VIEWVERIFICATION");
        this.logger.level = "debug";
	}
} */

class PumpVerification {
	constructor() {
        this.process = null;
        this.project = "pump";
        this.batch = null;
		this.logger = Log.getLogger("PUMP VIEWVERIFICATION");
        this.logger.level = "debug"
    }

    executeAll(callback) {
        const onExit = () => {
            this.logger.info("...Verification terminated.");
            callback();
        };

        this.kill();
        this.logger.info("Running Verification...");
        this.process = Process.spawn(BATCHES.PUMP.VIEWVERIFICATION.ALL, [], {
            cwd: Path.resolve(PATHS.SCRIPTS, this.project)
        });
        this.process.on("exit", onExit);
        return this.process;
    }

	execute(streamName, callback) {
        const onReadFile = (error, content) => {
            const collection = "<pre>" + content.toString().trim() + "</pre>";

            this.logger.info("...Results fetched.");
            callback(collection);
        };

        const onExit = () => {
            this.logger.info("...Verification executed.");
			callback();
        };

        this.kill();
        this.logger.info("Running Verification for %s...", streamName);
        this.process = Process.spawn(BATCHES.PUMP.VIEWVERIFICATION.SINGLE, [streamName], {
            cwd: Path.resolve(PATHS.SCRIPTS, this.project)
        });
        this.process.on("exit", onExit);
        return this.process;
    }
	
    kill() {
        if(this.process) this.process.kill();
    }
}

class ClusteringVerification {

}

module.exports = {
    AutoPilotVerification: new AutoPilotVerification(),
	PumpVerification: new PumpVerification(),
    ClusteringVerification: new ClusteringVerification()
};
