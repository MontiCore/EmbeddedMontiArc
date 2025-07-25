/* (c) https://github.com/MontiCore/monticore */
const Process = require("./process");
const {EXECUTABLES} = require("./constants");
const Log = require("log4js");

class Compiler {
    constructor() {
        this.logger = Log.getLogger("COMPILER");
    }

    compile(cpp) {
        
    }
}

module.exports = new Compiler();
