const Path = require("path");
const FileSystem = require("fs-extra");
const TaskList = require("listr");

const ROOT_FOLDER = Path.resolve(__dirname, "..");
const DOWNLOADS_FOLDER = Path.join(ROOT_FOLDER, "downloads");
const CACHE_FOLDER = Path.join(ROOT_FOLDER, "cache");

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
        getDeleteTask("Cache", CACHE_FOLDER)
    ], { concurrent: true });

    return tasks.run();
}

execute().catch(error => console.error(error.message));