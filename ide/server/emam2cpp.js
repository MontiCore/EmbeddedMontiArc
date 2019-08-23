/* (c) https://github.com/MontiCore/monticore */
const Process = require("./process");
const {EXECUTABLES, JARS} = require("./constants");
const Log = require("log4js");

class EMAM2CPP {
    constructor() {
        this.logger = Log.getLogger("EMAM2CPP");
    }

    generate(modelsDir, rootModel, outputDir, flags) {
        const arguments = [
            "-jar", JARS.EMAM2CPP,
            "--models-dir=" + modelsDir,
            "--root-model=" + rootModel,
            "--output-dir" + outputDir
        ].concat(flags);

        return Process.spawn(EXECUTABLES.JAVA, arguments);
    }
}

module.exports = new EMAM2CPP();
