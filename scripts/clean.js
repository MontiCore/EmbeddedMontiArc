//@ts-check
/*
 * IMPORTS
 */
const Log = require("log4js");
const FS = require("fs");
const Rimraf = require("rimraf");
const Settings = require("./settings");

/*
 * SHARED VARIABLES
 */
const Logger = Log.getLogger("CLEAN");

/*
 * LOGO
 */
require("./logo");

/*
 * CLEAN
 */

/*
 * STEP 1: DELETE DIRECTORIES
 */
Logger.level = "debug";

Logger.info("[STEP 1]: Deleting Directories...");

Logger.info("[STEP 1.1]: Deleting Output Path...");
if(FS.existsSync(Settings.PATHS.OUTPUT)) Rimraf.sync(Settings.PATHS.OUTPUT);
Logger.info("[STEP 1.1]: ...Output Path deleted.");

Logger.info("[STEP 1.2]: Deleting Target Path...");
if(FS.existsSync(Settings.PATHS.TARGET)) Rimraf.sync(Settings.PATHS.TARGET);
Logger.info("[STEP 1.2]: ...Target Path deleted.");

Logger.info("[STEP 1]: ...Directories deleted.");