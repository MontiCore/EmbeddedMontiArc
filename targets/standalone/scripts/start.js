//@ts-check
/*
 * IMPORTS
 */
const Express = require("express");
const Log = require("log4js");

/*
 * CONSTANTS
 */
const PORT = 3005;

/*
 * SHARED VARIABLES
 */
const App = Express();
const Logger = Log.getLogger("START");

Logger.level = "debug";

const options = {
    setHeaders: function(res, path) {
        if(path.endsWith(".wasm")) {
            res.set("Content-Type", "application/wasm");
        }
    }
};

/*
 * START
 */
Logger.info("Server is starting...");
App.use(Express.static("lib", options));
App.listen(PORT, () => Logger.info("...Server has been started: http://localhost:%s", PORT));