/* (c) https://github.com/MontiCore/monticore */
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

	startDistr(callback) {
		let process = null;

		const onSimDone = () => {
			this.logger.info("Simulation servers stopped!");
		}

		const onStdOut = (buffer) => {
            const data = buffer.toString();

            if(data.indexOf("BlueBox-2X Service") > -1) {
                this.logger.info("Simulation server has been started.");
                process.stdout.removeListener("data", onStdOut);
                callback();
            }else if(data.indexOf("User disconnected") > -1) {
				this.stopDistr(onSimDone);
			}
        };

        const onTimeout = () => {
            this.logger.info("Starting Distributed Simulation...");
            process = Process.spawn(BATCHES.AUTOPILOT.SIMULATION.START_DISTR, [], {
                cwd: Path.resolve(PATHS.SCRIPTS, "autopilot")
            });
			process.stdout.on("data", onStdOut);
        };

        const onStop = () => {
            setTimeout(onTimeout, 3000);
        };

		//stop previous server instance if existing, before launching a new server instance
        this.stopDistr(onStop);
	}

	stopDistr(callback) {
		let prToKill = [];

        const onNetstatOut = (chunk) => {
            let buffer = chunk.toString();
			const inputs = buffer.split("\r\n");
			//retrieve the first and last pids
			let first = inputs[0] || "";
			let last = "";
			for(let i=0; i<inputs.length; ++i)
				if(inputs[i].length > 0) last = inputs[i]; //keep last non-empty input

			//always stop first process, except for the DB (port 5432)
			const inp = first.match(/:5432/) ? last : first;
            const matches = inp.match(/.+?(\d+)$/);

			//save process ID to be killed
			if(matches && matches[1] !== "0")
				prToKill.push(matches[1]);
        };

        const onNetstatExit = () => {
			while(prToKill.length > 0) {
				let pid = prToKill.pop();
				this.logger.info("Stopping Simulation Server..." + pid);
                Process.spawn("taskkill", ["/PID", pid, "/F"]);
			}
            callback();
        };

		const server = Process.spawn("cmd", ["/c", "netstat", "-ano", '|', "findstr", ":80"]).on("exit", onNetstatExit);
        const db = Process.spawn("cmd", ["/c", "netstat", "-ano", '|', "findstr", ":5432"]).on("exit", onNetstatExit);
		const rmi = Process.spawn("cmd", ["/c", "netstat", "-ano", '|', "findstr", ":10101"]).on("exit", onNetstatExit);

		server.stdout.on("data", onNetstatOut);
        db.stdout.on("data", onNetstatOut);
		rmi.stdout.on("data", onNetstatOut);
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

class PacManSimulation {
    constructor() {
        this.logger = Log.getLogger("PACMAN");
        this.process = null;
        this.logger.level = "debug";
    }

    execute(callback) {
        this.logger.info("Executing PacMan...");

        const onExit = () => {
            this.logger.info("...PacMan executed");
            callback();
        };

        Process.spawn(BATCHES.PACMAN.SIMULATION, [], {
            cwd: Path.resolve(PATHS.SCRIPTS, "pacman")
        }).on("exit", onExit);
    }
}

class SuperMarioSimulation {
    constructor() {
        this.logger = Log.getLogger("SUPERMARIO");
        this.process = null;
        this.logger.level = "debug";
    }

    execute(callback) {
        this.logger.info("Executing SuperMario...");

        const onExit = () => {
            this.logger.info("...SuperMario executed");
            callback();
        };

        Process.spawn(BATCHES.SUPERMARIO.SIMULATION, [], {
            cwd: Path.resolve(PATHS.SCRIPTS, "supermario")
        }).on("exit", onExit);
    }
}

module.exports = {
    AutoPilotSimulation: new AutoPilotSimulation(),
    ClusteringSimulation: new ClusteringSimulation(),
    PacManSimulation: new PacManSimulation(),
    SuperMarioSimulation: new SuperMarioSimulation()
};
