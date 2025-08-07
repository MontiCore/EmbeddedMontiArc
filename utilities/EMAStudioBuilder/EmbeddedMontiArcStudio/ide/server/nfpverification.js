/* (c) https://github.com/MontiCore/monticore */
const Process = require("./process");
const {BATCHES, PATHS} = require("./constants");
const Log = require("log4js");
const Path = require("path");

class AbstractNFPVerification {
	constructor(project) {
        this.logger = null;
        this.process = null;
        this.project = project;
        this.batch = null;
    }

/*	handleResult(result, callback) {
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

        FileSystem.readdir(Path.resolve(PATHS.MODELS, "nfpverification"), onReaddir);
    } */
	
    execute(streamName, callback) {
	/*	const onResultsCollected = (collection) => {
            this.logger.info("...Results collected");
            callback(collection);
        }; */
		
        const onExit = () => {
            this.logger.info("...NFPVerification terminated.");
			//this.logger.info("Collecting Results...");
            //this.collectResults(onResultsCollected);
            callback();
        };

        this.kill();
        this.logger.info("Running NFPVerification for Sensors with %S...", streamName);
        this.process = Process.spawn(BATCHES.NFPVERIFICATION.TEST, [streamName], {
            cwd: Path.resolve(PATHS.SCRIPTS, this.project)
        });
        this.process.on("exit", onExit);
        return this.process;
    }
    kill() {
        if(this.process) this.process.kill();
    }
}

class NFPVerificatorTest extends AbstractNFPVerification {
    constructor() {
        super("nfpverification");
        this.logger = Log.getLogger("Start NFPVerification");
        this.logger.level = "debug";
    }
}

module.exports = {
	NFPVerificatorTest: new NFPVerificatorTest()
};
