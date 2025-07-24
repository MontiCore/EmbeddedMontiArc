/* (c) https://github.com/MontiCore/monticore */
const {BATCHES, PATHS} = require("./constants");
const Log = require("log4js");
const Path = require("path");
const Process = require("./process");
const FileSystem = require("fs");

class AbstractTest {
    handleResult(result, callback) {
        const path = Path.resolve(PATHS.TEST_RESULTS, result);

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

        FileSystem.readdir(PATHS.TEST_RESULTS, onReaddir);
    }
}

class AutoPilotTest extends AbstractTest {
    constructor() {
        super();
        this.logger = Log.getLogger("AUTOPILOT TEST");
        this.process = null;
        this.logger.level = "debug";
    }

    executeAll(callback) {
        const onResultsCollected = (collection) => {
            this.logger.info("...Results collected");
            callback(collection);
        };

        const onExit = () => {
            this.logger.info("...Tests executed.");
            this.logger.info("Collecting Results...");
            this.collectResults(onResultsCollected);
        };

        this.kill();
        this.logger.info("Running Tests...");
        this.process = Process.spawn(BATCHES.AUTOPILOT.TEST.ALL, [], {
            cwd: Path.resolve(PATHS.SCRIPTS, "autopilot")
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
            const path = Path.join(PATHS.TEST_RESULTS, streamName);

            this.logger.info("...Test executed.");
            this.logger.info("Fetching Results...");

            FileSystem.readFile(path, onReadFile);
        };

        this.kill();
        this.logger.info("Running Test for %s...", streamName);
        this.process = Process.spawn(BATCHES.AUTOPILOT.TEST.SINGLE, [streamName], {
            cwd: Path.resolve(PATHS.SCRIPTS, "autopilot")
        });
        this.process.on("exit", onExit);
        return this.process;
    }

    kill() {
        if(this.process) this.process.kill();
    }
}

class ClusteringTest {

}

module.exports = {
    AutoPilotTest: new AutoPilotTest(),
    ClusteringTest: new ClusteringTest()
};
