/* (c) https://github.com/MontiCore/monticore */
const Process = require("./process");
const Log = require("log4js");
const {BATCHES, PATHS} = require("./constants");
const Path = require("path");
const FileSystem = require("fs");

class AbstractOCLVerification {
    constructor(project, batch) {
        this.logger = null;
        this.process = null;
        this.project = project;
        this.batch = batch;
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

class CDVisualization extends AbstractOCLVerification {
    constructor() {
        super("oclverification", BATCHES.OCLVERIFICATION.VISUALIZECD);
        this.logger = Log.getLogger("Start OCLVerification");
        this.logger.level = "debug";
    }

    execute(path, callback) {
        const onExit = () => {
            this.logger.info("...OCL CLI has finished.");
            callback();
        };

        this.kill();
        this.logger.info("OCL CLI is visualizing CD...");

        var fullPath = PATHS.MODELS + "/oclverification" + path;
        this.logger.info("Reading model: " + fullPath);
        var cdString = FileSystem.readFileSync(fullPath, "UTF-8");
        cdString = cdString.replace(/(?:\r\n|\r|\n)/g, ' ');

        this.process = Process.spawn(this.batch, [cdString], {
            cwd: Path.resolve(PATHS.SCRIPTS, this.project)
        });
        this.process.on("exit", onExit);
        return this.process;
    }
}

class OCLChecking extends AbstractOCLVerification {
    constructor() {
        super("oclverification", BATCHES.OCLVERIFICATION.CHECKOCL);
        this.logger = Log.getLogger("Start OCLVerification");
        this.logger.level = "debug";
    }

    qualifyName(name) {
        var qualifiedName = name;
        qualifiedName = qualifiedName.replace(/\//g,".");
        qualifiedName = qualifiedName.replace(/\.ocl$/,"");
        if (qualifiedName.indexOf(".") == 0) {
            qualifiedName = qualifiedName.substring(1);
        }
        return qualifiedName;
    }

    execute(path, callback) {
        const onExit = () => {
            this.logger.info("...OCL CLI has finished.");
            var fullPath = PATHS.OCLVERIFICATION + "/data/result.txt";
            var result = FileSystem.readFileSync(fullPath, "UTF-8");
            callback(result);
        };

        this.kill();
        this.logger.info("OCL CLI is checking OCL types...");

        var qualifiedName = this.qualifyName(path);

        this.process = Process.spawn(this.batch, [qualifiedName], {
            cwd: Path.resolve(PATHS.SCRIPTS, this.project)
        });
        this.process.on("exit", onExit);
        return this.process;
    }
}

module.exports = {
	CDVisualization: new CDVisualization(),
	OCLChecking: new OCLChecking()
};
