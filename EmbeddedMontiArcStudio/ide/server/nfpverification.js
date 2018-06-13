const Process = require("./process");
const {BATCHES, PATHS} = require("./constants");
const Log = require("log4js");
const Path = require("path");

class AbstractNFPVerification {
	constructor(project, batch) {
        this.logger = null;
        this.process = null;
        this.project = project;
        this.batch = batch;
    }

    execute(streamName, callback) {
        const onExit = () => {
            this.logger.info("...NFPVerification terminated.");
            callback();
        };

        this.kill();
        this.logger.info("Running NFPVerification for %s...", streamName);
        this.process = Process.spawn(this.batch, [streamName], {
            cwd: Path.resolve(PATHS.SCRIPTS, this.project)
        });
        this.process.on("exit", onExit);
        return this.process;
    }

    kill() {
        if(this.process) this.process.kill();
    }
}

class NFPVerificatorTest1 extends AbstractNFPVerification {
    constructor() {
        super("nfpverification", BATCHES.NFPVERIFICATION.TEST1);
        this.logger = Log.getLogger("NFPVerification WITH RULE 1");
        this.logger.level = "debug";
    }
}

class NFPVerificatorTest2 extends AbstractNFPVerification {
    constructor() {
        super("nfpverification", BATCHES.NFPVERIFICATION.TEST2);
        this.logger = Log.getLogger("NFPVerification WITH RULE 2");
        this.logger.level = "debug";
    }
}

module.exports = {
	NFPVerificatorTest1: new NFPVerificatorTest1(),
	NFPVerificatorTest2: new NFPVerificatorTest2()
};