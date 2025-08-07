const Path = require("path");
const Glob = require("glob-promise");
const FileSystem = require("fs-extra");
const TaskList = require("listr");
const ChildProcess = require("child_process");


const MODULE_FOLDER = Path.resolve(__dirname, "..");
const DIST_FOLDER = Path.join(MODULE_FOLDER, "dist");
const RESOURCES_FOLDER = Path.join(MODULE_FOLDER, "cache", "resources");


function getExecutable() {
    if (process.platform === "win32") return "EmbeddedMontiArcStudio.exe";
    else if (process.platform === "linux") return "@emastudio/emastudio";
    else throw new Error("Unsupported Platform.");
}

function getPlatformFolder() {
    if (process.platform === "win32") return "windows";
    else if (process.platform === "linux") return "linux";
    else throw new Error("Unsupported Platform.");
}

async function getUnpackedFolder() {
    const unpackedPattern = Path.join(DIST_FOLDER, "*-unpacked");
    const unpackedGlobs = await Glob.promise(unpackedPattern);
    const unpackedFolder = unpackedGlobs[0];

    if (unpackedFolder) return unpackedFolder;
    else throw new Error("[ERROR]: Packaged Application does not exist. Please compile, build, and package the application first.");
}

async function getResourcesFolder(unpackedFolder) {
    const resourcesPattern = Path.join(unpackedFolder, "**", "resources");
    const resourcesGlobs = await Glob.promise(resourcesPattern);

    return resourcesGlobs[0];
}


async function getBuildInjectionCondition() {
    const pathExists = await FileSystem.pathExists(RESOURCES_FOLDER);

    return !pathExists;
}

async function getBuildInjectionTask(resourcesFolder) {
    return FileSystem.copy(RESOURCES_FOLDER, resourcesFolder, { overwrite: true });
}

function getInjectionTask(source, resourcesFolder) {
    const basename = Path.basename(source);

    return {
        title: `Injecting '${basename}'`,
        task: async () => {
            const destination = Path.join(resourcesFolder, basename);

            return FileSystem.copy(source, destination, { overwrite: true });
        }
    };
}

async function getSourcesInjectionTasks(resourcesFolder) {
    const pattern = Path.join(MODULE_FOLDER, "src", "!(common|windows|linux|electron)");
    const globs = await Glob.promise(pattern);

    return new TaskList(
        globs.map(glob => { return getInjectionTask(glob, resourcesFolder) }), { concurrent: 4 }
    );
}

async function getCommonInjectionTasks(resourcesFolder) {
    const pattern = Path.join(MODULE_FOLDER, "src", "common", '*');
    const globs = await Glob.promise(pattern);

    return new TaskList(
        globs.map(glob => { return getInjectionTask(glob, resourcesFolder) }), { concurrent: 4 }
    );
}

async function getPlatformInjectionTasks(resourcesFolder) {
    const platformFolder = getPlatformFolder();
    const pattern = Path.join(MODULE_FOLDER, "src", platformFolder, '*');
    const globs = await Glob.promise(pattern);

    return new TaskList(
        globs.map(glob => { return getInjectionTask(glob, resourcesFolder) }), { concurrent: 4 }
    );
}

function getInjectionTasks(resourcesFolder) {
    return new TaskList([{
        title: "Injecting Sources",
        task: () => { return getSourcesInjectionTasks(resourcesFolder); }
    }, {
        title: "Injecting Common Sources",
        task: () => { return getCommonInjectionTasks(resourcesFolder); }
    }, {
        title: "Injecting Platform Sources",
        task: () => { return getPlatformInjectionTasks(resourcesFolder); }
    }, {
        title: "Injecting Application",
        skip: () => { return getBuildInjectionCondition(); },
        task: () => { return getBuildInjectionTask(resourcesFolder); }
    }], { concurrent: 4 });
}

function getStartTask(unpackedFolder) {
    ChildProcess.spawn(getExecutable(), [], { cwd: unpackedFolder, detached: true });
}


async function execute() {
    const unpackedFolder = await getUnpackedFolder();
    const resourcesFolder = await getResourcesFolder(unpackedFolder);

    const tasks = new TaskList([{
        title: "Injection",
        task: () => { return getInjectionTasks(resourcesFolder); }
    }, {
        title: "Start",
        task: () => { return getStartTask(unpackedFolder); }
    }]);

    return tasks.run();
}

execute().catch(error => console.error(error.message));