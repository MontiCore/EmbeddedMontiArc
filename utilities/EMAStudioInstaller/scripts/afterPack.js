const Path = require("path");
const Glob = require("glob-promise");
const FileSystem = require("fs-extra");
const TaskList = require("listr");

const extract = require("extract-zip");

const ROOT_FOLDER = Path.resolve(__dirname, "..");
const DOWNLOADS_FOLDER = Path.join(ROOT_FOLDER, "target", "downloads");
const CONFIGS_FOLDER = Path.join(ROOT_FOLDER, "configs");
const CACHE_FOLDER = Path.join(ROOT_FOLDER, "target", "cache");
const RESOURCES_FOLDER = Path.join(CACHE_FOLDER, "resources");

function decompress(source, destination) {
    return new Promise((resolve, reject) => {
        extract(source, { dir: destination }, error => {
            if (error) reject(error);
            else resolve();
        });
    });
}

function getPrepareTask(dependency, resourcesFolder) {
    return {
        title: `Preparing: ${dependency.filename}`,
        task: async () => {
            const regex = /[\/=]([^\/=]+?.zip)/;
            const matches = regex.exec(dependency.from);

            dependency.skipped = !dependency.platforms || !dependency.platforms.includes(process.platform);
            dependency.filename = matches[1];
            dependency.downloadFolder = `${DOWNLOADS_FOLDER}/${dependency.to}`;
            dependency.downloadFile = `${dependency.downloadFolder}/${dependency.filename}`;
            dependency.decompressFolder = `${resourcesFolder}/../${dependency.to}`;
        }
    };
}

function getPrepareTasks(dependencies, resourcesFolder) {
    return new TaskList(
        dependencies.map(dependency => { return getPrepareTask(dependency, resourcesFolder); }), { concurrent: 4 }
    );
}

function getDecompressTask(dependency) {
    return {
        title: `Decompressing: ${dependency.filename} ${dependency.skipped ? "-> Skipped" : ''}`,
        task: async () => {
            if (dependency.skipped) return Promise.resolve();

            return decompress(`${dependency.downloadFile}`, `${dependency.decompressFolder}`);
        }
    };
}

function getDecompressTasks(dependencies) {
    return new TaskList(
        dependencies.map(dependency => { return getDecompressTask(dependency); }), { concurrent: 4 }
    );
}

function getMakeDirectoryTask(resourcesFolder, directory) {
    return {
        title: `Making Directory: ${directory}`,
        task: async () => { return FileSystem.mkdir(`${resourcesFolder}/${directory}`); }
    };
}

function getMakeDirectoriesTasks(resourcesFolder) {
    return new TaskList([
        getMakeDirectoryTask(resourcesFolder, "cpp"),
        getMakeDirectoryTask(resourcesFolder, "dll"),
        getMakeDirectoryTask(resourcesFolder, "exec"),
        getMakeDirectoryTask(resourcesFolder, "testInfo"),
        getMakeDirectoryTask(resourcesFolder, "testResults"),
        getMakeDirectoryTask(resourcesFolder, "tests-cpp"),
    ], { concurrent: 4 });
}

// outDir: dist
// appOutDir: win-unpacked
module.exports = async function(context) {
    const dependenciesFile = Path.join(CONFIGS_FOLDER, "dependencies.json");
    const dependencies = await FileSystem.readJSON(dependenciesFile);

    const resourcesPattern = Path.join(context.appOutDir, "**", "resources");
    const resourcesGlobs = await Glob.promise(resourcesPattern);
    const resourcesFolder = resourcesGlobs[0];

    const tasks = new TaskList([{
        title: "Preparing Dependencies",
        task: () => { return getPrepareTasks(dependencies, resourcesFolder); }
    }, {
        title: "Decompressing Dependencies",
        task: () => { return getDecompressTasks(dependencies); }
    }, {
        title: "Creating Empty Directories",
        task: () => { return getMakeDirectoriesTasks(resourcesFolder); }
    }, {
        title: "Copying Application",
        task: async () => { return FileSystem.copy(RESOURCES_FOLDER, resourcesFolder); }
    }]);

    return tasks.run();
};