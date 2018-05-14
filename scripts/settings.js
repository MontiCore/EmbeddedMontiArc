//@ts-check
/*
 * IMPORTS
 */
const Path = require("path");
const Util = require("util");

/*
 * SETTINGS
 */
const PATHS = {};

PATHS.ROOT = Path.resolve(__dirname, "..");
PATHS.TARGET = Path.join(PATHS.ROOT, "target");
PATHS.OUTPUT = Path.join(PATHS.ROOT, "docs");
PATHS.THEIA = Path.join(PATHS.ROOT, "theia");
PATHS.CONFIG = Path.join(PATHS.ROOT, "configs");
PATHS.NODE_MODULES = Path.join(PATHS.ROOT, "node_modules");


const CONFIGS = {};

CONFIGS.TYPE_DOC = Path.join(PATHS.CONFIG, "typedoc.json");


const SCRIPTS = {};

SCRIPTS.WEBPACK = Path.join(PATHS.ROOT, "node_modules", "webpack", "bin", "webpack.js");
SCRIPTS.YARN = Path.join(PATHS.ROOT, "node_modules", "yarn", "bin", "yarn.js");
SCRIPTS.LERNA = Path.join(PATHS.ROOT, "node_modules", "lerna", "bin", "lerna.js");
SCRIPTS.TYPE_DOC = Path.join(PATHS.ROOT, "node_modules", "typedoc", "bin", "typedoc");


const THEIA = {};

THEIA.USERNAME = "theia-ide";
THEIA.PROJECT = "theia";
THEIA.REFERENCE = "a52fab7d14e6d556647b5e202749a48aefd65cfe";

THEIA.URL = Util.format(
    "https://github.com/%s/%s/archive/%s.zip",
    THEIA.USERNAME, THEIA.PROJECT, THEIA.REFERENCE
);

THEIA.ZIP = Path.join(PATHS.TARGET, "theia.zip");

THEIA.UNUSED_PACKAGES = ["cpp", "java", "python"];


const NAMES = {};

NAMES.TS_CONFIG = "compile.tsconfig.json";


/*
 * EXPORTS
 */
module.exports = {
    PATHS: PATHS,
    CONFIGS: CONFIGS,
    SCRIPTS: SCRIPTS,
    THEIA: THEIA,
    NAMES: NAMES
};