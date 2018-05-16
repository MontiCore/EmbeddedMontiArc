const Express                                           = require("express");
const Path                                              = require("path");
const {PATHS, URLS, OPTIONS}                            = require("./constants");
const Chrome                                            = require("./chrome");
const {AutoPilotSimulation, ClusteringSimulation}       = require("./simulations");
const {AutoPilotVisualization, ClusteringVisualization, PumpVisualization} = require("./visualizations");
const {AutoPilotReporting, ClusteringReporting, PumpReporting}         = require("./reportings");
const {AutoPilotReportingWS, ClusteringReportingWS}     = require("./reportings");
const {AutoPilotVerification, ClusteringVerification, PumpVerification} = require("./viewverification");
const Log                                               = require("log4js");
const {AutoPilotTest, ClusteringTest}                   = require("./tests");
const ModelUpdater                                      = require("./models-updater");
const FileUpload                                        = require("express-fileupload");
const Jimp                                              = require("jimp");
const FileSystem                                        = require("fs");

const App = Express();
const Logger = Log.getLogger("APPLICATION");


Logger.level = "debug";


App.use("/m", Express.static(PATHS.MODELS));
App.use("/r", Express.static(Path.resolve(PATHS.REPORTING, "report")));
App.use("/c", Express.static(PATHS.CLUSTER_FIDDLE));
App.use("/v", Express.static(Path.resolve(PATHS.VISUALIZATION, "SVG")));
App.use("/h", Express.static(PATHS.VIDEOS));
App.use('/',  Express.static(Path.resolve(PATHS.IDE, "client"), OPTIONS.STATIC));
App.use("/vv", Express.static(Path.resolve(PATHS.VIEWVERIFICATION, "WitnessSVG")));

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

App.post("/services/autopilot/simulate-distr", function(request, response) {
	function onStart() {
        Chrome.open(URLS.AUTOPILOT.DISTR_SIM);
        response.end();
    }

    AutoPilotSimulation.startDistr(onStart);
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

App.post("/services/autopilot/viewverification/all", function(request, response) {
	function onExecuted() {
		// Select the source folder.
		//var myFolder = PATHS.VIEWVERIFICATION_OUTPUT;
		var files = FileSystem.readdirSync(PATHS.VIEWVERIFICATION_OUTPUT);

		/*// If a valid folder is selected
		if (myFolder != null) {
			var files = new Array();

			// Get all files matching the pattern
			files = myFolder.getFiles();*/

		if (files.length > 0) {
			// Get the destination to save the files
			for (i = 0; i < files.length; i++) {
				//var sourceDoc = app.open(files[i]); // returns the document object
				//var title = sourceDoc.name;
				var title = files[i];
				if(!title.substring(0,title.length-5).includes(".") && !title.startsWith("icons"))
					Chrome.open(URLS.SHARED + "/vv/" + title);
				//Close the Source Document
				//sourceDoc.close(SaveOptions.DONOTSAVECHANGES);
			}
		}
		else {
			alert('No matching files found');
		}
		response.end();
	}

	AutoPilotVerification.executeAll(onExecuted);
});

App.post("/services/autopilot/viewverification/single", function(request, response) {
    const body = request.body;

    function onExecuted() {
        		// Select the source folder.
		//var myFolder = PATHS.VIEWVERIFICATION_OUTPUT;
		var files = FileSystem.readdirSync(PATHS.VIEWVERIFICATION_OUTPUT);

		/*// If a valid folder is selected
		if (myFolder != null) {
			var files = new Array();

			// Get all files matching the pattern
			files = myFolder.getFiles();*/

		if (files.length > 0) {
			// Get the destination to save the files
			for (i = 0; i < files.length; i++) {
				//var sourceDoc = app.open(files[i]); // returns the document object
				//var title = sourceDoc.name;
				var title = files[i];
				var name = body.name;
				var parts = name.split("/");
				var titlestart = parts[parts.length-1].substring(0,parts[parts.length-1].length-4);
				if(title.startsWith(titlestart + "Witness"))
					Chrome.open(URLS.SHARED + "/vv/" + title);
				//Close the Source Document
				//sourceDoc.close(SaveOptions.DONOTSAVECHANGES);
			}
		}
		else {
			alert('No matching files found');
		}
		response.end();
    }

    AutoPilotVerification.execute("model\\autopilot" + body.name.replace(/\//g, "\\"), onExecuted);
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

App.post("/services/clustering/viewverification", function(request, response) {

});

App.post("/services/pump/viewverification/single", function(request, response) {
    const body = request.body;

    function onExecuted() {
        		// Select the source folder.
		//var myFolder = PATHS.VIEWVERIFICATION_OUTPUT;
		var files = FileSystem.readdirSync(PATHS.VIEWVERIFICATION_OUTPUT);

		/*// If a valid folder is selected
		if (myFolder != null) {
			var files = new Array();

			// Get all files matching the pattern
			files = myFolder.getFiles();*/

		if (files.length > 0) {
			// Get the destination to save the files
			for (i = 0; i < files.length; i++) {
				//var sourceDoc = app.open(files[i]); // returns the document object
				//var title = sourceDoc.name;
				var title = files[i];
				var name = body.name;
				var parts = name.split("/");
				var titlestart = parts[parts.length-1].substring(0,parts[parts.length-1].length-4);
				if(title.startsWith(titlestart + "Witness"))
					Chrome.open(URLS.SHARED + "/vv/" + title);
				//Close the Source Document
				//sourceDoc.close(SaveOptions.DONOTSAVECHANGES);
			}
		}  
		else {
			alert('No matching files found');
		} 
		response.end();
    }
	
    PumpVerification.execute("model\\pump" + body.name.replace(/\//g, "\\"), onExecuted);
});

App.post("/services/pump/viewverification/all", function(request, response) {
	function onExecuted() {
		// Select the source folder.
		//var myFolder = PATHS.VIEWVERIFICATION_OUTPUT;
		var files = FileSystem.readdirSync(PATHS.VIEWVERIFICATION_OUTPUT);

		/*// If a valid folder is selected
		if (myFolder != null) {
			var files = new Array();

			// Get all files matching the pattern
			files = myFolder.getFiles();*/

		if (files.length > 0) {
			// Get the destination to save the files
			for (i = 0; i < files.length; i++) {
				//var sourceDoc = app.open(files[i]); // returns the document object
				//var title = sourceDoc.name;
				var title = files[i];
				if(!title.substring(0,title.length-5).includes(".") && !title.startsWith("icons"))
					Chrome.open(URLS.SHARED + "/vv/" + title);
				//Close the Source Document
				//sourceDoc.close(SaveOptions.DONOTSAVECHANGES);
			}
		}  
		else {
			alert('No matching files found');
		} 
		response.end();
	}

	PumpVerification.executeAll(onExecuted);
});

App.post("/services/pump/visualize", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/v/pumpStationExample.pumpStation.html");
        response.end();
    }

    PumpVisualization.execute(onExecuted);
});

App.post("/services/pump/test", function(request, response) {

});

App.post("/services/pump/report", function(request, response) {
function onExecuted() {
        Chrome.open(URLS.SHARED + "/r/report.html?ide=false");
        response.end();
    }

    PumpReporting.execute(onExecuted);
});

App.post("/services/pump/reportWS", function(request, response) {

});

App.post("/services/pump/simulate", function(request, response) {

});

module.exports = App;
