/* (c) https://github.com/MontiCore/monticore */
const Log = require("log4js");
const rimraf = require("rimraf");
const mkdirp = require("mkdirp");
const {PATHS} = require("./constants");
const Path = require("path");
const FileSystem = require("fs");

class ModelsUpdater {
    constructor() {
        this.logger = Log.getLogger("MODELS-UPDATER");
    }

    update(project, collection, callback) {
        const radix = Path.resolve(PATHS.MODELS, project);

        const onRimraf = (error) => {
            if(error) callback(error);
            else this.doUpdate(collection, radix, callback);
        };

        rimraf(radix, onRimraf);
    }

    doUpdate(collection, radix,  callback) {
        const entry = collection.shift();

        if(entry) {
            const file = entry.path;
            const content = entry.content;
            const path = Path.resolve(radix, '.' + file);

            const onWriteFile = (error) => {
                if(error) callback(error);
                else this.doUpdate(collection, radix, callback);
            };

            this.writeFile(path, content, onWriteFile);
        } else {
            callback(null);
        }
    }

    writeFile(path, content, callback) {
        const onMkdirp = (error) => {
            if(error) callback(error);
            else FileSystem.writeFile(path, content, callback);
        };

        mkdirp(Path.dirname(path), onMkdirp);
    }
}

module.exports = new ModelsUpdater();
