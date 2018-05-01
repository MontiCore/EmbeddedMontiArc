//@ts-check

/*===========================================================
 * 1.  CLEAR TARGET FOLDER
 * 2.  CLEAR THEIA FOLDER
 * 3.  DOWNLOAD THEIA
 * 4.  UNZIP THEIA
 * 5.  COPY PACKAGES TO THEIA
 * 6.  DELETE UNUSED PACKAGES
 * 7.  RUN YARN INSTALL
 * 8.  COPY TSCONFIG TO @THEIA/EXT-SCRIPTS
 * 9.  RUN LERNA EXEC -- TSC --PROJECT COMPILE.TSCONFIG.JSON
 * 10. DELETE TARGET FOLDER
===========================================================*/

/*
 * IMPORTS
 */
const Log = require("log4js");
const Settings = require("./settings");
const FS = require("fs-extra");
const Request = require("request");
const Unzip = require("unzip");
const Glob = require("glob");
const Path = require("path");
const ChildProcess = require("child_process");
const Rimraf = require("rimraf");

/*
 * SHARED VARIABLES
 */
const Logger = Log.getLogger("INSTALL");

/*
 * LOGO
 */
require("./logo");

/*
 * INSTALL
 */
function onTargetDeletedFinal() {
    Logger.info("[STEP 10]: ...Target Folder deleted.");
}

function onPackagesCompiled(code) {
    if(code !== 0) return Logger.error("An error occurred while copying the TSConfig for @theia/ext-scripts: ", code);

    Logger.info("[STEP 9]: ...Theia Packages have been compiled.");
    Logger.info("[STEP 10]: Deleting Target Folder...");
    Rimraf(Settings.PATHS.TARGET, onTargetDeletedFinal);
}

function onTSConfigCopied(error) {
    const options = { cwd: Settings.PATHS.ROOT, stdio: "inherit" };
    const args = [Settings.SCRIPTS.LERNA, "exec", "--scope", "@theia/*", "--", "tsc", "--project", Settings.NAMES.TS_CONFIG];

    if(error) return Logger.error("An error occurred while copying the TSConfig for @theia/ext-scripts: ", error);

    Logger.info("[STEP 8]: ...TSConfig has been copied to @theia/ext-scripts.");
    Logger.info("[STEP 9]: Compiling Theia Packages...");
    ChildProcess.spawn("node", args, options).on("exit", onPackagesCompiled);
}

function onTSConfigGlobbed(error, config) {
    const destination = Path.join(Settings.PATHS.THEIA, "ext-scripts", Settings.NAMES.TS_CONFIG);

    if(error) return Logger.error("An error occurred while searching for the Configs: ", error);

    Logger.info("[STEP 8]: Copying TSConfig to @theia/ext-scripts...");
    FS.copy(config[0], destination, onTSConfigCopied);
}

function onYarnInstalled(code) {
    const pattern = Path.join(Settings.PATHS.THEIA, "cli", Settings.NAMES.TS_CONFIG);

    if(code !== 0) return Logger.error("...An error occurred while installing the Packages: ", code);

    Logger.info("[STEP 7]: ...Packages installed");
    Glob(pattern, onTSConfigGlobbed);
}

function onUnusedPackagesRemoved() {
    const options = { cwd: Settings.PATHS.ROOT, stdio: "inherit" };
    const args = [Settings.SCRIPTS.YARN, "install"];

    Logger.info("[STEP 6]: ...Unused Packages deleted.");
    Logger.info("[STEP 7]: Installing Packages...");
    ChildProcess.spawn("node", args, options).on("exit", onYarnInstalled);
}

function onPackagesCopied() {
    Logger.info("[STEP 5]: ...Packages copied.");
    Logger.info("[STEP 6]: Deleting Unused Packages...");

    for(const upackage of Settings.THEIA.UNUSED_PACKAGES) {
        const path = Path.join(Settings.PATHS.THEIA, upackage);

        Rimraf.sync(path);
    }

    onUnusedPackagesRemoved();
}

function onPackagesGlobbed(error, directories) {
    const options = { "overwrite": true };

    if(error) return Logger.error("...An error occurred while searching for packages: ", error);

    Logger.info("[STEP 5]: Copying Packages...");
    Promise.all(
        directories.map(
            directory => FS.copy(directory, Settings.PATHS.THEIA, options)
        )
    ).then(onPackagesCopied);
}

function onUnzipClose() {
    const pattern = Path.join(Settings.PATHS.TARGET, "**", "+(packages|dev-packages)");

    Logger.info("[STEP 4]: ...Theia unpacked.");
    Glob(pattern, onPackagesGlobbed);
}

function onDownloadClose() {
    const options = { path: Settings.PATHS.TARGET };
    const extractor = Unzip.Extract(options);
    const readStream = FS.createReadStream(Settings.THEIA.ZIP).pipe(extractor);

    Logger.info("[STEP 3]: ...Theia downloaded.");
    Logger.info("[STEP 4]: Unpacking Theia...");
    readStream.on("close", onUnzipClose);
}

function onTheiaCreated(error) {
    const writeStream = FS.createWriteStream(Settings.THEIA.ZIP);

    if(error) return Logger.error("An error occurred while clearing the theia folder:", error);

    Logger.info("[STEP 2]: ...Theia Folder has been cleared.");
    Logger.info("[STEP 3]: Downloading Theia...");
    Request(Settings.THEIA.URL).pipe(writeStream);
    writeStream.on("close", onDownloadClose);
}

function onTheiaExists(exists) {
    if(exists) FS.emptyDir(Settings.PATHS.THEIA, onTheiaCreated);
    else FS.ensureDir(Settings.PATHS.THEIA, onTheiaCreated);
}

function onTargetCreated(error) {
    if(error) return Logger.error("An error occurred while clearing the target folder:", error);

    Logger.info("[STEP 1]: ...Target Folder has been cleared.");
    Logger.info("[STEP 2]: Clearing Theia Folder...");
    FS.pathExists(Settings.PATHS.THEIA, onTheiaExists);
}

function onTargetExists(exists) {
    if(exists) FS.emptyDir(Settings.PATHS.TARGET, onTargetCreated);
    else FS.ensureDir(Settings.PATHS.TARGET, onTargetCreated);
}

Logger.level = "debug";
Logger.info("[STEP 1]: Clearing Target Folder...");
FS.pathExists(Settings.PATHS.TARGET, onTargetExists);