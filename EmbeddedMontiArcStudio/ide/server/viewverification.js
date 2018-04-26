const {BATCHES, PATHS} = require("./constants");
const Log = require("log4js");
const Path = require("path");
const Process = require("./process");
//const FileSystem = require("fs");

/*class AbstractVerification {
    handleResult(result, callback) {
        const path = Path.resolve(PATHS.VIEWVERIFICATION_RESULT, result);

        FileSystem.readFile(path, callback);
    }

    doHandleResults(results, collection, callback) {
        const result = results.shift();

        const onHandle = (error, content) => {
            if(error) return callback(error);

            collection += "<pre>"
                + content
                    .toString()
                    .trim()
                + "</pre>";
            this.doHandleResults(results, collection, callback);
        };

        if(result) this.handleResult(result, onHandle);
        else callback(collection);
    }

    handleResults(results, callback) {
        this.doHandleResults(results, "", callback);
    }

    collectResults(callback) {
        const onReaddir = (error, results) => {
            if(error) callback(error);
            else this.handleResults(results, callback);
        };

        FileSystem.readdir(PATHS.VIEWVERIFICATION_RESULT, onReaddir);
    }
} */

class AbstractVerification {
	constructor(project) {
        this.logger = null;
        this.process = null;
        this.project = project;
        this.batch = null;
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

class AutoPilotVerification extends AbstractVerification {
	constructor() {
		super("autopilot", BATCHES.AUTOPILOT.VIEWVERIFICATION);
		this.logger = Log.getLogger("AUTOPILOT VIEWVERIFICATION");
        this.logger.level = "debug";
	}
}

class ClusteringVerification {

}

module.exports = {
    AutoPilotVerification: new AutoPilotVerification(),
    ClusteringVerification: new ClusteringVerification()
};