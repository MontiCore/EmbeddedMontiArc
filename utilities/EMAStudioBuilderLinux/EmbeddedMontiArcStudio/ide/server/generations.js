/* (c) https://github.com/MontiCore/monticore */
const Process = require("./process");
const Log = require("log4js");
const {BATCHES, PATHS} = require("./constants");
const Path = require("path");

class AbstractEMAM2WASM  {
    constructor(project, batch) {
        this.logger = null;
        this.process = null;
        this.project = project;
        this.batch = batch;
    }

    execute(callback, tab) {
        const onExit = () => {
            this.logger.info("...Web Assembly Generation has finished.");
            callback();
        };

        this.kill();
        this.logger.info("Generation is running...");

        this.process = Process.spawn(this.batch, [this.project, this.qualifyName(tab)], {
            cwd: Path.resolve(PATHS.SCRIPTS, this.project)
        });
        this.process.on("exit", onExit);
        return this.process;
    }

    kill() {
        if(this.process) this.process.kill();
    }

    qualifyName(name) {
        var newName = name;
        newName = newName.replace(/\//g,".").replace(".emam","").replace(".ema","");
        if (newName.indexOf(".") == 0) {
            newName = newName.substring(1);
        }
        var packageName = newName.substring(0, newName.lastIndexOf("."));
        var modelName = newName.substring(newName.lastIndexOf(".") + 1);
        modelName = ("" + modelName.charAt(0)).toLowerCase() + modelName.substring(1);

        return packageName + "." + modelName;
    }
}

class PacmanGeneration extends AbstractEMAM2WASM {
    constructor() {
        super("pacman", BATCHES.PACMAN.EMAM2WASM_GEN);
        this.logger = Log.getLogger("CLUSTERING REPORTING WITH STREAMS");
        this.logger.level = "debug";
    }
}

module.exports = {
    PacmanGeneration: new PacmanGeneration(),
};
