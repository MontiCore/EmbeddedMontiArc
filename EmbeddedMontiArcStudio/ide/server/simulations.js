const Process = require("./process");
const Log = require("log4js");
const {BATCHES, PATHS, EXECUTABLES} = require("./constants");
const Path = require("path");

class AutoPilotSimulation {
    constructor() {
        this.logger = Log.getLogger("AUTOPILOT SIMULATION");
        this.logger.level = "debug";
    }

    start(callback) {
        let process = null;

        const onStdOut = (buffer) => {
            const data = buffer.toString();

            if(data.indexOf("World is initialling") > -1) {
                this.logger.info("...Simulation has been started.");
                process.stdout.removeListener("data", onStdOut);
                callback();
            }
        };

        const onTimeout = () => {
            this.logger.info("Starting Simulation...");
            process = Process.spawn(BATCHES.AUTOPILOT.SIMULATION.START, [], {
                cwd: Path.resolve(PATHS.SCRIPTS, "autopilot")
            });
            process.stdout.on("data", onStdOut);
        };

        const onStop = () => {
            setTimeout(onTimeout, 3000);
        };

        this.stop(onStop);
    }

    /*stop(callback) {
        this.logger.info("Stopping Simulation...");
        Process.spawn(BATCHES.AUTOPILOT.SIMULATION.STOP, [], {
            cwd: Path.resolve(PATHS.SCRIPTS, "autopilot")
        }).on("exit", callback);
    }*/

    stop(callback) {
        let buffer = "";

        const onTaskkillExit = () => {
            callback();
        };

        const onNetstatOut = (chunk) => {
            buffer = chunk.toString();
        };

        const onNetstatExit = () => {
            const matches = buffer.match(/.+?(\d+)\r\n/);

            if(matches && matches[1] !== "0") {
                this.logger.info("Stopping Simulation...");
                Process.spawn("taskkill", ["/PID", matches[1], "/F"]).on("exit", onTaskkillExit);
            } else {
                callback();
            }
        };

        const process = Process.spawn("cmd", ["/c", "netstat", "-ano", '|', "findstr", ":8080"]).on("exit", onNetstatExit);

        process.stdout.on("data", onNetstatOut);
    }
}

class ClusteringSimulation {
    constructor() {
        this.logger = Log.getLogger("CLUSTERING");
        this.process = null;
        this.logger.level = "debug";
    }

    prepare(callback) {
        this.logger.info("Generating Clusterer...");

        const onExit = () => {
            this.logger.info("...Clusterer generated");
            callback();
        };

        Process.spawn(BATCHES.CLUSTERING.SIMULATION.GENERATE, [], {
            cwd: Path.resolve(PATHS.SCRIPTS, "clustering")
        }).on("exit", onExit);
    }

    execute(callback) {
        this.logger.info("Executing Clusterer...");

        const onExit = () => {
            this.logger.info("...Clusterer executed");
            callback();
        };

        Process.spawn(BATCHES.CLUSTERING.SIMULATION.EXECUTE, [], {
            cwd: Path.resolve(PATHS.SCRIPTS, "clustering")
        }).on("exit", onExit);
    }
}

module.exports = {
    AutoPilotSimulation: new AutoPilotSimulation(),
    ClusteringSimulation: new ClusteringSimulation()
};