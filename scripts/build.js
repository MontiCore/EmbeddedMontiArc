//@ts-check
/*
 * IMPORTS
 */
const Log = require("log4js");
const Settings = require("./settings");
const FS = require("fs");
const Path = require("path");
const ChildProcess = require("child_process");

/*
 * SHARED VARIABLES
 */
const Logger = Log.getLogger("BUILD");

/*
 * LOGO
 */
require("./logo");

/*
 * BUILD
 */

/*
 * STEP 1: COMPILE PACKAGES
 */
require("./compile");

/*
 * STEP 2: DELETE DIRECTORIES
 */
require("./clean");

/*
 * STEP 3: RECREATE DIRECTORIES
 */
Logger.level = "debug";
Logger.info("[STEP 1]: Creating directories...");
FS.mkdirSync(Settings.PATHS.OUTPUT);
Logger.info("[STEP 1]: ...Directories created.");

/*
 * STEP 4: RUN WEBPACK
 */
const options = { "stdio": "inherit" };
const arguments = [
    Settings.SCRIPTS.WEBPACK,
    "--config", Path.join(Settings.PATHS.CONFIG, "webpack.config.js"),
    "--display-error-details"
];

Logger.info("[STEP 2]: Running Webpack...");
ChildProcess.execFileSync("node", arguments, options);
Logger.info("[STEP 2]: ...Webpack finished.");