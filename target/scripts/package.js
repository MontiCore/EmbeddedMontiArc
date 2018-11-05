const Path = require("path");
const FileSystem = require("fs-extra");
const TaskList = require("listr");
const Archiver = require("archiver");

const MODULE_FOLDER = Path.resolve(__dirname, "..");

const LIB_FOLDER = Path.join(MODULE_FOLDER, "lib");

const DIST_FOLDER = Path.join(MODULE_FOLDER, "dist");
const IDE_ZIP = Path.join(DIST_FOLDER, "ide.zip");

function getCompressTask() {
    return {
        title: "Packaging Application",
        task: async () => {
            await FileSystem.ensureDir(DIST_FOLDER);

            return new Promise(resolve => {
                const archive = Archiver("zip");
                const writeStream = FileSystem.createWriteStream(IDE_ZIP);

                archive.directory(LIB_FOLDER, '.');
                writeStream.on("close", resolve);
                archive.pipe(writeStream);
                archive.finalize();
            });
        }
    };
}

async function execute() {
    const tasks = new TaskList([ getCompressTask() ]);

    return tasks.run();
}

execute().catch(error => console.error(error.message));