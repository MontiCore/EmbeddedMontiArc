/* (c) https://github.com/MontiCore/monticore */
const Express                                           = require("express");
const Path                                              = require("path");
const {PATHS, URLS, OPTIONS}                            = require("./constants");
const Chrome                                            = require("./chrome");
const {AutoPilotSimulation, ClusteringSimulation, PacManSimulation, SuperMarioSimulation} = require("./simulations");
const {AutoPilotVisualization, ClusteringVisualization, PumpVisualization, PacManVisualization, SuperMarioVisualization,AutoPilotVisualizationRes1,AutoPilotVisualizationRes2} = require("./visualizations");
const {AutoPilotReporting, ClusteringReporting, PumpReporting, PacManReporting, SuperMarioReporting} = require("./reportings");
const {AutoPilotReportingWS, ClusteringReportingWS, PacManReportingWS, SuperMarioReportingWS} = require("./reportings");
const {AutoPilotVerification, ClusteringVerification, PumpVerification} = require("./viewverification");
const {PacmanGeneration, SuperMarioGeneration}          = require("./generations");
const {NFPVerificatorTest} = require("./nfpverification");
const {CDVisualization, OCLChecking} = require("./oclverification");
const Log                                               = require("log4js");
const {AutoPilotTest, ClusteringTest, PacManTest, SuperMarioTest} = require("./tests");
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
App.use("/pp", Express.static(PATHS.PACMAN_PLAY));
App.use("/ps", Express.static(PATHS.PACMAN_SIMULATE));
App.use("/mp", Express.static(PATHS.SUPERMARIO_PLAY));
App.use("/ms", Express.static(PATHS.SUPERMARIO_SIMULATE));
App.use('/',  Express.static(Path.resolve(PATHS.IDE, "client"), OPTIONS.STATIC));
App.use("/vv", Express.static(Path.resolve(PATHS.VIEWVERIFICATION, "WitnessSVG")));
App.use("/nfp", Express.static(PATHS.NFPVERIFICATION_RESULT))
App.use("/ocl", Express.static(PATHS.OCLVERIFICATION));

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
		Chrome.open(URLS.SHARED + "/v/index.html");
		response.end();
	}

	AutoPilotVisualization.execute(onExecuted);
});


App.post("/services/autopilot/visualizeRes1", function(request, response) {
	function onExecuted() {
		Chrome.open(URLS.SHARED + "/v/de.rwth.armin.modeling.autopilot.motion.calculatePidError_extended.html");
		response.end();
	}

	AutoPilotVisualizationRes1.execute(onExecuted);
});


App.post("/services/autopilot/visualizeRes2", function(request, response) {
	function onExecuted() {
		Chrome.open(URLS.SHARED + "/v/de.rwth.armin.modeling.autopilot.motion.calculateEngineAndBrakes_extended.html");
		response.end();
	}

	AutoPilotVisualizationRes2.execute(onExecuted);
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
        Chrome.open(URLS.SHARED + "/v/index.html");
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
        Chrome.open(URLS.SHARED + "/v/index.html");
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

App.post("/services/pacman/report", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/r/report.html?ide=false");
        response.end();
    }

    PacManReporting.execute(onExecuted);
});

App.post("/services/pacman/reportWS", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/r/report.html?ide=false&streams=true");
        response.end();
    }

    PacManReportingWS.execute(onExecuted);
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

App.post("/services/pacman/visualize", function(request, response) {
	function onExecuted() {
		Chrome.open(URLS.SHARED + "/v/index.html");
		response.end();
	}

	PacManVisualization.execute(onExecuted);
});

App.post("/services/pacman/test/all", function(request, response) {
	function onTested(results) {
		response.send(results);
	}

	PacManTest.executeAll(onTested);
});

App.post("/services/pacman/test/single", function(request, response) {
    const body = request.body;

    function onTested(results) {
        response.send(results);
    }

    PacManTest.execute(body.streamName, onTested);
});

App.post("/services/supermario/report", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/r/report.html?ide=false");
        response.end();
    }

    SuperMarioReporting.execute(onExecuted);
});

App.post("/services/supermario/reportWS", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/r/report.html?ide=false&streams=true");
        response.end();
    }

    SuperMarioReportingWS.execute(onExecuted);
});

App.post("/services/supermario/play", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/mp");
        response.end();
    }
    onExecuted();
});

App.post("/services/supermario/emam2wasmGen", function(request, response) {
    function onExecuted() {
        response.end();
    }

    var tab = request.body.tab;

    SuperMarioGeneration.execute(onExecuted, tab);
});

App.post("/services/supermario/simulate", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/ms/simulation.html");
        response.end();
    }
    SuperMarioSimulation.execute(onExecuted);
});

App.post("/services/supermario/visualize", function(request, response) {
	function onExecuted() {
		Chrome.open(URLS.SHARED + "/v/index.html");
		response.end();
	}

	SuperMarioVisualization.execute(onExecuted);
});

App.post("/services/supermario/test/all", function(request, response) {
	function onTested(results) {
		response.send(results);
	}

	SuperMarioTest.executeAll(onTested);
});

App.post("/services/supermario/test/single", function(request, response) {
    const body = request.body;

    function onTested(results) {
        response.send(results);
    }

    SuperMarioTest.execute(body.streamName, onTested);
});

App.post("/services/nfpverification/test", function(request, response) {
	const body = request.body;
	
	function doNothing(){}
	
	function onUpdated() {
		var targetfolder = "witnesses_" + body.name.replace(/\//g, ".").substring(1,body.name.length-4) + "_example.model.Sensors"; 
		var files = FileSystem.readdirSync(Path.resolve(PATHS.MODELS, "nfpverification\\target", targetfolder))
		
		if (files.length > 0) {
			for (i = 0; i < files.length; i++) {
				/*if(files[i].inclides(".ema")) {
					var witnessPath = Path.resolve(PATHS.MODELS, "nfpverification\\target", targetfolder, files[i]);
					var witness = FileSystem.readFileSync(witnessPath, "UTF-8");
					var witnessParts = witness.split("\n");
					
					var witnesstxt = "";
					var changed = false;
					for (j = 0; j < witnessParts.length; j++) {
						if(!changed && witnessParts[j].startsWith("component")) {
							witnesstxt += "component " + files[i].substring(0, files[i].length-4) + " { \n" 
							changed = true;
						}
						else {
							witnesstxt += witnessParts[j] + "\n"
						}
					}
					
					FileSystem.appendFile(witnessPath, witnesstxt, doNothing);
				}*/
				//ModelUpdater.writeFile(Path.resolve(PATHS.MODELS, "nfpverification\\target", targetfolder), files[i], doNothing);
				//FileSystem.writeFile(Path.resolve(PATHS.MODELS, "nfpverification\\target", targetfolder), files[i], doNothing);
				ModelUpdater.update("NFPVerification", files, doNothing)
			}		
			var txtFile = Path.resolve(PATHS.MODELS, "nfpverification\\target", targetfolder, "__WITNESS_OVERVIEW__.txt");
			var file = FileSystem.readFileSync(txtFile, "UTF-8");
			//file = file.replace(/(?:\r\n|\r|\n)/g, '\n');
			var fileParts = file.split("\n");
			//var str = "";
			var str = "<!DOCTYPE html> \n <html><body><header>Witness Overview</header> \n";
			for (i = 0; i < fileParts.length; i++) {
				var txt = fileParts[i].replace(/\t/g, '&emsp;');
				txt = txt.replace(/\ /g, '&nbsp;'); 
				var lineParts = fileParts[i].replace(/\t/g, ' ').split(" ");
				var comp = lineParts[1]
				str += "<a href="+URLS.SHARED + "/nfp/" + comp + ".html>" + txt + "</a> <br> \n";
				//lineParts[1]; fileParts[i]
			}
			str += "</body></html>"

			FileSystem.appendFile(Path.resolve(PATHS.NFPVERIFICATION_RESULT,"result.html"), str, doNothing);
		}  
		else {
			alert('No matching files found');
		} 

		Chrome.open(URLS.SHARED + "/nfp/result.html");
		response.end();
	}

	NFPVerificatorTest.execute(body.name.replace(/\//g, ".").substring(1,body.name.length-4), onUpdated);	
});

App.post("/services/oclverification/visualizeCD", function(request, response) {
    function onExecuted() {
        Chrome.open(URLS.SHARED + "/ocl/visualizeCD.html?ide=false");
        response.end();
    }

    var path = request.body.path;
    CDVisualization.execute(path, onExecuted);
});

App.post("/services/oclverification/checkOCL", function(request, response) {
    function onExecuted(result) {
        response.send(result);
    }


    var path = request.body.path;
    OCLChecking.execute(path, onExecuted);
});

module.exports = App;
