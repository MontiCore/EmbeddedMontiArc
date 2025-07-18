const Path = require("path");
const FileSystem = require("fs-extra");
const TaskList = require("listr");

const MODULE_FOLDER = Path.resolve(__dirname, "..");
const DOWNLOADS_FOLDER = Path.join(MODULE_FOLDER, "downloads");
const CACHE_FOLDER = Path.join(MODULE_FOLDER, "cache");
const DIST_FOLDER = Path.join(MODULE_FOLDER, "dist");

function getDeleteTask(name, path) {
    return {
        title: `Deleting ${name}`,
        task: async () => {
            const exists = await FileSystem.pathExists(path);

            if (exists) return FileSystem.remove(path);
        }
    };
}

async function execute() {
    const tasks = new TaskList([
        getDeleteTask("Downloads", DOWNLOADS_FOLDER),
        getDeleteTask("Cache", CACHE_FOLDER),
        getDeleteTask("Distribution", DIST_FOLDER)
    ], { concurrent: true });

    return tasks.run();
}

execute().catch(error => console.error(error.message));