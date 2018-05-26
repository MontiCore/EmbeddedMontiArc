const Express                                                       = require("express");
const Path                                                          = require("path");
const {PATHS, URLS, OPTIONS}                                        = require("./constants");
const Chrome                                                        = require("./chrome");
const {AutoPilotSimulation, ClusteringSimulation, PacManSimulation} = require("./simulations");
const {AutoPilotVisualization, ClusteringVisualization}             = require("./visualizations");
const {AutoPilotReporting, ClusteringReporting, PacManReporting}    = require("./reportings");
const {AutoPilotReportingWS, ClusteringReportingWS}                 = require("./reportings");
const {PacmanGeneration}                                            = require("./generations");
const Log                                                           = require("log4js");
const {AutoPilotTest, ClusteringTest}                               = require("./tests");
const ModelUpdater                                                  = require("./models-updater");
const FileUpload                                                    = require("express-fileupload");
const Jimp                                                          = require("jimp");
const FileSystem                                                    = require("fs");

const App = Express();
const Logger = Log.getLogger("APPLICATION");


Logger.level = "debug";


App.use("/m", Express.static(PATHS.MODELS));
App.use("/r", Express.static(Path.resolve(PATHS.REPORTING, "report")));
App.use("/c", Express.static(PATHS.CLUSTER_FIDDLE));
App.use("/v", Express.static(Path.resolve(PATHS.VISUALIZATION, "SVG")));
App.use("/h", Express.static(PATHS.VIDEOS));
App.use("/pp", Express.static(PATHS.PACMAN_PLAY));
App.use("/ps", Express.static(PATHS.PACMAN_SIMULATE));
App.use('/',  Express.static(Path.resolve(PATHS.IDE, "client"), OPTIONS.STATIC));

App.use("/services/clustering/simulate/cluster", FileUpload());
App.use("/services", Express.json());

App.post("/services/:project/update-models", function(request, response) {
	const body = request.body;
	const params = request.params;

	function onUpdated() {
	    response.end();
    }

	ModelUpdater.update(params.project, body, onUpdated);
});

App.post("/services/autopilot/simulate", function(request, response) {
	function onStart() {
        Chrome.open(URLS.AUTOPILOT.SIMULATION);
        response.end();
    }

    AutoPilotSimulation.start(onStart);
});

App.post("/services/autopilot/visualize", function(request, response) {
	function onExecuted() {
		Chrome.open(URLS.SHARED + "/v/de.rwth.armin.modeling.autopilot.autopilot.html");
		response.end();
	}

	AutoPilotVisualization.execute(onExecuted);
});

App.post("/services/autopilot/report", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/r/report.html?ide=false");
        response.end();
    }

    AutoPilotReporting.execute(onExecuted);
});

App.post("/services/autopilot/reportWS", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/r/report.html?ide=false&streams=true");
        response.end();
    }

    AutoPilotReportingWS.execute(onExecuted);
});

App.post("/services/autopilot/test/all", function(request, response) {
	function onTested(results) {
		response.send(results);
	}

	AutoPilotTest.executeAll(onTested);
});

App.post("/services/autopilot/test/single", function(request, response) {
    const body = request.body;

    function onTested(results) {
        response.send(results);
    }

    AutoPilotTest.execute(body.streamName, onTested);
});

App.post("/services/clustering/simulate", function(request, response) {
	function onPrepared() {
        Chrome.open(URLS.SHARED + "/c");
	    response.end();
    }

    ClusteringSimulation.prepare(onPrepared);
});

App.post("/services/clustering/simulate/cluster", function(request, response) {
    const image = request.files.file;
    const imagePath = Path.join(PATHS.EXEC, "img.bmp");
    const resultPath = Path.join(PATHS.EXEC, "result.bmp");

    function onCopyFile() {
        response.end();
    }

    function onExecuted() {
        FileSystem.copyFile(resultPath, Path.join(PATHS.CLUSTER_FIDDLE, "img.bmp"), onCopyFile);
    }

    function onImageRead(error, image) {
        if(error) return response.end();

        image.resize(50, 50).write(imagePath);
        ClusteringSimulation.execute(onExecuted);
    }

    function onMV(error) {
        if(error) response.end();
        else Jimp.read(imagePath, onImageRead);
    }

    image.mv(imagePath, onMV);
});

App.post("/services/clustering/visualize", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/v/detection.spectralClusterer.html");
        response.end();
    }

    ClusteringVisualization.execute(onExecuted);
});

App.post("/services/clustering/report", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/r/report.html?ide=false");
        response.end();
    }

    ClusteringReporting.execute(onExecuted);
});

App.post("/services/clustering/reportWS", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/r/reportWS.html?ide=false&streams=true");
        response.end();
    }

    ClusteringReportingWS.execute(onExecuted);
});

App.post("/services/clustering/test", function(request, response) {

});

App.post("/services/pacman/report", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/r/report.html?ide=false");
        response.end();
    }

    PacManReporting.execute(onExecuted);
});

App.post("/services/pacman/play", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/pp");
        response.end();
    }
    onExecuted();
});

App.post("/services/pacman/emam2wasmGen", function(request, response) {
    function onExecuted() {
        response.end();
    }

    var tab = request.body.tab;

    PacmanGeneration.execute(onExecuted, tab);
});

App.post("/services/pacman/simulate", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/ps/simulation.html");
        response.end();
    }
    PacManSimulation.execute(onExecuted);
});

#############

App.post("/services/supermario/report", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/r/report.html?ide=false");
        response.end();
    }

    SuperMarioReporting.execute(onExecuted);
});

App.post("/services/supermario/play", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/pp");
        response.end();
    }
    onExecuted();
});

App.post("/services/supermario/emam2wasmGen", function(request, response) {
    function onExecuted() {
        response.end();
    }

    var tab = request.body.tab;

    PacmanGeneration.execute(onExecuted, tab);
});

App.post("/services/supermario/simulate", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/ps/simulation.html");
        response.end();
    }
    SuperMarioSimulation.execute(onExecuted);
});

module.exports = App;