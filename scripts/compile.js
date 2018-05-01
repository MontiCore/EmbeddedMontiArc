/*
 * IMPORTS
 */
const Settings = require("./settings");
const Log = require("log4js");
const ChildProcess = require("child_process");

/*
 * SHARED VARIABLES
 */
const Logger = Log.getLogger("COMPILE");

/*
 * LOGO
 */
require("./logo");

/*
 * COMPILE
 */
const options = { cwd: Settings.PATHS.ROOT, stdio: "inherit" };
const args = [Settings.SCRIPTS.LERNA, "exec", "--scope", "@elysium/*", "--", "tsc", "--project", Settings.NAMES.TS_CONFIG];

Logger.level = "debug";

Logger.info("[STEP 1]: Compiling Packages...");
ChildProcess.spawnSync("node", args, options);
Logger.info("[STEP 1]: ...Packages compiled.");