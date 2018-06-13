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

    execute1(streamName, callback) {
        const onExit = () => {
            this.logger.info("...NFPVerification terminated.");
            callback();
        };

        this.kill();
        this.logger.info("Running NFPVerification with rule 1 for %s...", streamName);
        this.process = Process.spawn(BATCHES.NFPVERIFICATION.TEST1, [streamName], {
            cwd: Path.resolve(PATHS.SCRIPTS, this.project)
        });
        this.process.on("exit", onExit);
        return this.process;
    }

	execute2(streamName, callback) {
        const onExit = () => {
            this.logger.info("...NFPVerification terminated.");
            callback();
        };

        this.kill();
        this.logger.info("Running NFPVerification with rule 2 for %s...", streamName);
        this.process = Process.spawn(BATCHES.NFPVERIFICATION.TEST2, [streamName], {
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