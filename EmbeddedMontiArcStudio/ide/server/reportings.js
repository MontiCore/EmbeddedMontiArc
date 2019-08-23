/* (c) https://github.com/MontiCore/monticore */
const Process = require("./process");
const Log = require("log4js");
const {BATCHES, PATHS} = require("./constants");
const Path = require("path");

class AbstractReporting {
    constructor(project, batch) {
        this.logger = null;
        this.process = null;
        this.project = project;
        this.batch = batch;
    }

    execute(callback) {
        const onExit = () => {
            this.logger.info("...Reporting has finished.");
            callback();
        };

        this.kill();
        this.logger.info("Reporting is running...");
        this.process = Process.spawn(this.batch, [], {
            cwd: Path.resolve(PATHS.SCRIPTS, this.project)
        });
        this.process.on("exit", onExit);
        return this.process;
    }

    kill() {
        if(this.process) this.process.kill();
    }
}

class AutoPilotReporting extends AbstractReporting {
    constructor() {
        super("autopilot", BATCHES.AUTOPILOT.REPORTING);
        this.logger = Log.getLogger("AUTOPILOT REPORTING");
        this.logger.level = "debug";
    }
}

class AutoPilotReportingWS extends AbstractReporting {
    constructor() {
        super("autopilot", BATCHES.AUTOPILOT.REPORTING_STREAM);
        this.logger = Log.getLogger("AUTOPILOT REPORTING WITH STREAMS");
        this.logger.level = "debug";
    }
}

class ClusteringReporting extends AbstractReporting {
    constructor() {
        super("clustering", BATCHES.CLUSTERING.REPORTING);
        this.logger = Log.getLogger("CLUSTERING REPORTING");
        this.logger.level = "debug";
    }
}

class ClusteringReportingWS extends AbstractReporting {
    constructor() {
        super("clustering", BATCHES.CLUSTERING.REPORTING_STREAM);
        this.logger = Log.getLogger("CLUSTERING REPORTING WITH STREAMS");
        this.logger.level = "debug";
    }
}

class PumpReporting extends AbstractReporting {
    constructor() {
        super("pump", BATCHES.PUMP.REPORTING);
        this.logger = Log.getLogger("PUMP REPORTING");
        this.logger.level = "debug";
    }
}

class PacManReporting extends AbstractReporting {
    constructor() {
        super("pacman", BATCHES.PACMAN.REPORTING);
        this.logger = Log.getLogger("PACMAN REPORTING");
        this.logger.level = "debug";
    }
}

module.exports = {
    AutoPilotReporting: new AutoPilotReporting(),
    AutoPilotReportingWS: new AutoPilotReportingWS,
    ClusteringReporting: new ClusteringReporting(),
    ClusteringReportingWS: new ClusteringReportingWS(),
    PumpReporting: new PumpReporting(),
    PacManReporting: new PacManReporting()
};
