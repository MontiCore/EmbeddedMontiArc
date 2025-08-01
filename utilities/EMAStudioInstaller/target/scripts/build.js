const Path = require("path");
const FileSystem = require("fs-extra");
const TaskList = require("listr");

const MODULE_FOLDER = Path.resolve(__dirname, "..");
const ROOT_FOLDER = Path.resolve(MODULE_FOLDER, "..");

const MODULES_SOURCE = Path.join(ROOT_FOLDER, "node_modules");
const ELYSIUM_SOURCE = Path.join(MODULES_SOURCE, "@elysium");
const EMASTUDIO_SOURCE = Path.join(MODULES_SOURCE, "@emastudio");
const SOURCES_SOURCE = Path.join(MODULE_FOLDER, "src-gen");
const LIB_SOURCE = Path.join(MODULE_FOLDER, "lib");

const APP_FOLDER = Path.join(MODULE_FOLDER, "cache", "resources", "app");
const MODULES_TARGET = Path.join(APP_FOLDER, "node_modules");
const ELYSIUM_TARGET = Path.join(MODULES_TARGET, "@elysium");
const EMASTUDIO_TARGET = Path.join(MODULES_TARGET, "@emastudio");
const SOURCES_TARGET = Path.join(APP_FOLDER, "src-gen");
const LIB_TARGET = Path.join(APP_FOLDER, "lib");

const OPTIONS = {
    overwrite: true,
    dereference: true,
    filter: src => {
        return src.indexOf(Path.join("@emastudio", "emastudio")) === -1;
    }
};

function getCopyTask(title, source, destination) {
    return {
        title: title,
        task: async () => { await FileSystem.copy(source, destination, OPTIONS); }
    };
}

function getCopyTasks() {
    return {
        title: "Copying Resources",
        task: async () => {
            await FileSystem.emptyDir(APP_FOLDER);
            await FileSystem.ensureDir(MODULES_TARGET);

            return new TaskList([
                getCopyTask("Copying Sources", SOURCES_SOURCE, SOURCES_TARGET),
                getCopyTask("Copying Library", LIB_SOURCE, LIB_TARGET),
                getCopyTask("Copying EmbeddedMontiArcStudio Modules", EMASTUDIO_SOURCE, EMASTUDIO_TARGET),
                getCopyTask("Copying Elysium Modules", ELYSIUM_SOURCE, ELYSIUM_TARGET)
            ]);
        }
    };
}

async function execute() {
    const tasks = new TaskList([
        getCopyTasks()
    ]);

    return tasks.run();
}

execute().catch(error => console.error(error.message));
