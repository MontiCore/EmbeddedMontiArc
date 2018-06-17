const Process = require("./process");
const Log = require("log4js");
const {BATCHES, PATHS} = require("./constants");
const Path = require("path");

class AbstractOCLVerification {
    constructor(project, batch, fs) {
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
    constructor(fs) {
        super("oclverification", BATCHES.OCLVERIFICATION.VISUALIZECD);
        this.logger = Log.getLogger("Start OCLVerification");
        this.logger.level = "debug";
    }

    execute(callback, path, FileSystem) {
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

module.exports = {
	CDVisualization: new CDVisualization()
};