/* (c) https://github.com/MontiCore/monticore */
const Process = require("./process");
const {BATCHES, PATHS} = require("./constants");
const Log = require("log4js");
const Path = require("path");

/*class Visualization {
	constructor() {
		this.logger = Log.getLogger("VISUALIZATION");
	}

	prepare(outputPath, callback) {
		const onAccess = (error) => {
			if(error) FileSystem.rmdir(outputPath, callback);
			else callback(null);
		};

		this.logger("Preparing Visualization...");
		FileSystem.access(outputPath, onAccess);
	}
	
	generate(input, modelPath, outputPath, callback) {
		const onPrepared = (error) => {
			if(error) this.logger.error("An error occurred while generating the Visualization: %j.", error);
			else this.doGenerate(input, modelPath, outputPath, callback);
		};

		this.logger("Generating Visualization...");
		this.prepare(outputPath, onPrepared);
	}

	doGenerate(input, modelPath, outputPath) {
		const arguments = [
			"-jar", JARS.SVGGENERATOR,
			"--input", input,
			"--modelPath", modelPath,
			"--recursiveDrawing", true,
			"--outputPath", outputPath
		];

		return Process.spawn(EXECUTABLES.JAVA, arguments);
	}
}

module.exports = new Visualization();*/

class AbstractVisualization {
	constructor(project, batch) {
        this.logger = null;
        this.process = null;
        this.project = project;
        this.batch = batch;
    }

    execute(callback) {
        const onExit = () => {
            this.logger.info("...Visualization terminated.");
            callback();
        };

        this.kill();
        this.logger.info("Running Visualization...");
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

class AutoPilotVisualization extends AbstractVisualization {
	constructor() {
		super("autopilot", BATCHES.AUTOPILOT.VISUALIZATION);
		this.logger = Log.getLogger("AUTOPILOT VISUALIZATION");
        this.logger.level = "debug";
	}
}

class AutoPilotVisualizationRes1 extends AbstractVisualization {
	constructor() {
		super("autopilot", BATCHES.AUTOPILOT.VISUALIZATIONRES1);
		this.logger = Log.getLogger("CALCULATE PID ERROR VISUALIZATION WITH RESOURCE INTERFACE");
        this.logger.level = "debug";
	}
}

class AutoPilotVisualizationRes2 extends AbstractVisualization {
	constructor() {
		super("autopilot", BATCHES.AUTOPILOT.VISUALIZATIONRES2);
		this.logger = Log.getLogger("CALCULATE ENGINE AND BRAKES VISUALIZATION WITH RESOURCE INTERFACE");
        this.logger.level = "debug";
	}
}

class ClusteringVisualization extends AbstractVisualization {
    constructor() {
        super("clustering", BATCHES.CLUSTERING.VISUALIZATION);
        this.logger = Log.getLogger("CLUSTERING VISUALIZATION");
        this.logger.level = "debug";
    }
}

class PumpVisualization extends AbstractVisualization {
    constructor() {
        super("pump", BATCHES.PUMP.VISUALIZATION);
        this.logger = Log.getLogger("PUMP VISUALIZATION");
        this.logger.level = "debug";
    }
}

class PacManVisualization extends AbstractVisualization {
    constructor() {
        super("pacman", BATCHES.PACMAN.VISUALIZATION);
        this.logger = Log.getLogger("PACMAN VISUALIZATION");
        this.logger.level = "debug";
    }
}

class SuperMarioVisualization extends AbstractVisualization {
    constructor() {
        super("supermario", BATCHES.SUPERMARIO.VISUALIZATION);
        this.logger = Log.getLogger("SUPERMARIO VISUALIZATION");
        this.logger.level = "debug";
    }
}

module.exports = {
	AutoPilotVisualization: new AutoPilotVisualization(),
	AutoPilotVisualizationRes1: new AutoPilotVisualizationRes1(),
	AutoPilotVisualizationRes2: new AutoPilotVisualizationRes2(),
	PumpVisualization: new PumpVisualization(),
	ClusteringVisualization: new ClusteringVisualization(),
    PacManVisualization: new PacManVisualization(),
    SuperMarioVisualization: new SuperMarioVisualization()
};
