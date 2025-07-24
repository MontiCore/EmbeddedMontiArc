/* (c) https://github.com/MontiCore/monticore */
const Log = require("log4js");
const Path = require("path");
const {PATHS} = require("./constants");
const Util = require("util");
const FileSystem = require("fs");

class Archiver {
    constructor() {
        this.archiver = require("archiver");
        this.logger = Log.getLogger("ARCHIVER");
        this.logger.level = "debug";
    }

    zip(project, callback) {
        const zipFile = Util.format("%s.zip", project);
        const zipPath = Path.join(PATHS.MODELS, zipFile);
        const projectPath = Path.join(PATHS.MODELS, project);
        const output = FileSystem.createWriteStream(zipPath);
        const archive = this.archiver("zip", { zlib: { level: 9 }});

        output.on("end", callback);
        output.on("close", callback);

        this.logger.info("Zipping %s...", project);
        archive.pipe(output);
        archive.directory(projectPath, false);
        archive.finalize();
    }
}

module.exports = new Archiver();
