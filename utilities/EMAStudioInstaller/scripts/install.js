const Path = require("path");
const FileSystem = require("fs-extra");
const TaskList = require("listr");

const fetch = require("download");

const ROOT_FOLDER = Path.resolve(__dirname, "..");
const CONFIGS_FOLDER = Path.join(ROOT_FOLDER, "configs");
const DOWNLOADS_FOLDER = Path.join(ROOT_FOLDER, "target", "downloads");
const CACHE_FOLDER = Path.join(ROOT_FOLDER, "target", "cache");

function getPrepareTask(dependency) {
    return {
        title: `Preparing: ${dependency.filename}`,
        task: async () => {
            const regex = /[\/=]([^\/=]+?.zip)/;
            const matches = regex.exec(dependency.from);

            dependency.skipped = !dependency.platforms || !dependency.platforms.includes(process.platform);
            dependency.filename = matches[1];
            dependency.downloadFolder = `${DOWNLOADS_FOLDER}/${dependency.to}`;
            dependency.downloadFile = `${dependency.downloadFolder}/${dependency.filename}`;
            dependency.decompressFolder = `${CACHE_FOLDER}/${dependency.to}`;
        }
    };
}

function getPrepareTasks(dependencies) {
    return new TaskList(
        dependencies.map(dependency => { return getPrepareTask(dependency); }), { concurrent: 4 }
    );
}

function getCheckTask(dependency) {
    return {
        title: `Checking: ${dependency.filename}`,
        task: async () => {
            if (dependency.skipped) return Promise.resolve();

            dependency.skipped = await FileSystem.pathExists(dependency.downloadFile);
        }
    };
}

function getCheckTasks(dependencies) {
    return new TaskList(
        dependencies.map(dependency => { return getCheckTask(dependency); }), { concurrent: 4 }
    );
}

function getDownloadTask(dependency) {
    return {
        title: `Downloading: ${dependency.filename} ${dependency.skipped ? "-> Skipped" : ''}`,
        task: async () => {
            if (dependency.skipped) return Promise.resolve();

            await FileSystem.ensureDir(dependency.downloadFolder);
            return fetch(dependency.from, dependency.downloadFolder, { filename: dependency.filename });
        }
    };
}

function getDownloadTasks(dependencies) {
    return new TaskList(
        dependencies.map(dependency => { return getDownloadTask(dependency); }), { concurrent: 4 }
    );
}

async function execute() {
    const dependenciesFile = Path.join(CONFIGS_FOLDER, "dependencies.json");
    const dependencies = await FileSystem.readJSON(dependenciesFile);

    const tasks = new TaskList([{
        title: "Preparing Dependencies",
        task: () => { return getPrepareTasks(dependencies); }
    }, {
        title: "Checking Dependencies",
        task: () => { return getCheckTasks(dependencies); }
    }, {
        title: "Downloading Dependencies",
        task: () => { return getDownloadTasks(dependencies); }
    }]);

    return tasks.run();
}

execute().catch(error => console.error(error.message));