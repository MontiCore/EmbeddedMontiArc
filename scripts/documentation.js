/*
 * IMPORTS
 */
const Settings = require("./settings");
const Log = require("log4js");
const ChildProcess = require("child_process");
const Path = require("path");
const Glob = require("glob");
const FS = require("fs-extra");
const Rimraf = require("rimraf");

/*
 * SHARED VARIABLES
 */
const Logger = Log.getLogger("DOCUMENTATION");

Logger.level = "debug";

/*
 * LOGO
 */
require("./logo");

/*
 * TYPE-DOC
 */

/*
 * STEP 1: REMOVE OLD DOCUMENTATIONS FROM MAIN DOCS FOLDER
 */
Logger.info("[STEP 1]: Removing Old Documentations from Output Folder...");

let pattern = Path.join(Settings.PATHS.OUTPUT, "+(packages|plugins)", "**", "docs");
let globs = Glob.sync(pattern);

for (const glob of globs) {
    Rimraf.sync(glob);
}

Logger.info("[STEP 1]: ...Old Documentations removed from Output Folder.");

/*
 * STEP 2: GENERATE NEW DOCUMENTATIONS
 */
const options = { cwd: Settings.PATHS.ROOT, stdio: "inherit" };
const args = [
    Settings.SCRIPTS.LERNA, "exec", "--scope", "@elysium/*", "--",
    "typedoc", "--options", Settings.CONFIGS.TYPE_DOC, "--tsconfig", Settings.NAMES.TS_CONFIG, '.'
];

Logger.info("[STEP 2]: Generating New Documentations...");
ChildProcess.spawnSync("node", args, options);
Logger.info("[STEP 2]: ...New Documentations generated.");

/*
 * STEP 3: REMOVE LOCAL PATH
 */
const localPath = Path.posix.normalize(Settings.PATHS.NODE_MODULES) + Path.posix.sep;
const regex = new RegExp(localPath, 'g');

Logger.info("[STEP 3]: Removing Local Path...");

pattern = Path.join(Settings.PATHS.ROOT, "+(packages|plugins)", "**", "docs", "**", "*.html");
globs = Glob.sync(pattern);

for (const glob of globs) {
    const content = FS.readFileSync(glob).toString();
    const replacement = content.replace(regex, '');

    FS.writeFileSync(glob, replacement);
}

Logger.info("[STEP 3]: ...Local Path removed.");

/*
 * STEP 4: MOVE TO MAIN DOCS FOLDER
 */
Logger.info("[STEP 4]: Moving Documentations to Output Folder...");

pattern = Path.join(Settings.PATHS.ROOT, "+(packages|plugins)", "**", "docs");
globs = Glob.sync(pattern);

for (const glob of globs) {
    const globNormalized = Path.normalize(glob);
    const target = globNormalized.replace(Settings.PATHS.ROOT, Settings.PATHS.OUTPUT);

    FS.moveSync(globNormalized, target);
}

Logger.info("[STEP 4]: ...Documentations moved to Output Folder.");