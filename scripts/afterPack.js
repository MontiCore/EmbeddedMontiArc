const Path = require("path");
const Glob = require("glob-promise");
const FileSystem = require("fs-extra");
const TaskList = require("listr");

const ROOT_FOLDER = Path.resolve(__dirname, "..");
const CACHE_FOLDER = Path.join(ROOT_FOLDER, "target", "cache");
const RESOURCES_FOLDER = Path.join(CACHE_FOLDER, "resources");

function getCopyResourcesTask(resourcesFolder) {
    return {
        title: "Copying Resources",
        task: async () => {
            return FileSystem.copy(RESOURCES_FOLDER, resourcesFolder);
        }
    };
}

function getMakeDirectoriesTask(resourcesFolder) {
    return {
        title: "Creating Empty Directories",
        task: async () => {
            return Promise.all([
                FileSystem.mkdir(`${resourcesFolder}/cpp`),
                FileSystem.mkdir(`${resourcesFolder}/dll`),
                FileSystem.mkdir(`${resourcesFolder}/exec`),
                FileSystem.mkdir(`${resourcesFolder}/testInfo`),
                FileSystem.mkdir(`${resourcesFolder}/testResults`),
                FileSystem.mkdir(`${resourcesFolder}/tests-cpp`)
            ]);
        }
    };
}

// outDir: dist
// appOutDir: win-unpacked
module.exports = async function(context) {
    const resourcesPattern = Path.join(context.appOutDir, "**", "resources");
    const resourcesGlobs = await Glob.promise(resourcesPattern);
    const resourcesFolder = resourcesGlobs[0];

    const tasks = new TaskList([
        getCopyResourcesTask(resourcesFolder),
        getMakeDirectoriesTask(resourcesFolder)
    ], { concurrent: true });

    return tasks.run();
};