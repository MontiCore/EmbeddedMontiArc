/* (c) https://github.com/MontiCore/monticore */
/*
 * IMPORTS
 */
const Path = require("path");
const Util = require("util");
const findPort = require("find-port-sync");

/*
 * PATHS
 */
const PATHS = {};

PATHS.HOME           = Path.resolve(__dirname, "..", "..");
PATHS.ARMADILLO      = Path.resolve(PATHS.HOME, "armadillo-8.500.1-linux");
PATHS.MXNET      = Path.resolve(PATHS.HOME, "incubator-mxnet");
PATHS.CHROME         = Path.resolve(PATHS.HOME, "chrome-linux");
PATHS.CLUSTER_FIDDLE = Path.resolve(PATHS.HOME, "cluster-fiddle");
PATHS.CLASSIFIER     = Path.resolve(PATHS.HOME, "classifier");
PATHS.IDE            = Path.resolve(PATHS.HOME, "ide");
PATHS.JDK            = Path.resolve(PATHS.HOME, "java-linux");
PATHS.MINGW          = Path.resolve(PATHS.HOME, "mingw64");
PATHS.GPP            = Path.resolve(PATHS.HOME, "g++-linux");
PATHS.MODELS         = Path.resolve(PATHS.HOME, "model");
PATHS.NODEJS         = Path.resolve(PATHS.HOME, "nodejs");
PATHS.OCTAVE         = Path.resolve(PATHS.HOME, "octave-4.2.1");
PATHS.REPORTING      = Path.resolve(PATHS.HOME, "reporting");
PATHS.SCRIPTS        = Path.resolve(PATHS.HOME, "scripts");
PATHS.VIDEOS         = Path.resolve(PATHS.HOME, "videos");
PATHS.SIMULATION     = Path.resolve(PATHS.HOME, "apache-tomcat-9.0.5");
PATHS.VISUALIZATION  = Path.resolve(PATHS.HOME, "visualisation");
PATHS.TEST_RESULTS   = Path.resolve(PATHS.HOME, "testResults");
PATHS.EXEC           = Path.resolve(PATHS.HOME, "exec");
PATHS.PACMAN_PLAY    = Path.resolve(PATHS.HOME, "pacman");
PATHS.PACMAN_SIMULATE= Path.resolve(PATHS.HOME, "pacman");

PATHS.VIEWVERIFICATION = Path.resolve(PATHS.HOME, "viewverification");
PATHS.VIEWVERIFICATION_RESULT = Path.resolve(PATHS.VIEWVERIFICATION, "result");
PATHS.VIEWVERIFICATION_OUTPUT = Path.resolve(PATHS.VIEWVERIFICATION, "WitnessSVG");


/*
 * EXECUTABLES
 */
const EXECUTABLES = {};

EXECUTABLES.CHROME    = Path.resolve(PATHS.CHROME, "google-chrome");
EXECUTABLES.JAVA      = Path.resolve(PATHS.JDK, "bin", "java");
EXECUTABLES.GPP       = Path.resolve(PATHS.GPP, "bin", "g++");

/*
 * JARS
 */
const JARS = {};

JARS.EMAM2EMA     = Path.resolve(PATHS.VISUALIZATION, "emam2ema.jar");
JARS.SVGGENERATOR = Path.resolve(PATHS.VISUALIZATION, "embeddedmontiarc-svggenerator.jar");
JARS.EMAM2CPP     = Path.resolve(PATHS.HOME, "emam2cpp.jar");
JARS.EMADL2CPP    = Path.resolve(PATHS.HOME, "emadl2cpp.jar");
JARS.REPORTING    = Path.resolve(PATHS.REPORTING, "reporting.jar");

JARS.VIEWVERIFICATION = Path.resolve(PATHS.VIEWVERIFICATION, "view-verification-0.0.2-SNAPSHOT-jar-with-dependencies");
/*
 * BATCHES
 */
const BATCHES = {};

BATCHES.AUTOPILOT  = {};
BATCHES.CLUSTERING = {};
BATCHES.PUMP = {};
BATCHES.PACMAN     = {};
BATCHES.INTERSECTION = {};
BATCHES.CLASSIFIER = {};

BATCHES.AUTOPILOT.SIMULATION       			= {};
BATCHES.AUTOPILOT.SIMULATION.START 			= Path.resolve(PATHS.SCRIPTS, "autopilot", "simulate.start.sh");
BATCHES.AUTOPILOT.SIMULATION.START_DISTR 	= Path.resolve(PATHS.SCRIPTS, "autopilot", "simulate.start_distr.bat");
BATCHES.AUTOPILOT.SIMULATION.STOP  			= Path.resolve(PATHS.SCRIPTS, "autopilot", "simulate.stop.bat");
BATCHES.AUTOPILOT.VISUALIZATION    			= Path.resolve(PATHS.SCRIPTS, "autopilot", "visualize.sh");
BATCHES.AUTOPILOT.REPORTING        			= Path.resolve(PATHS.SCRIPTS, "autopilot", "report.bat");
BATCHES.AUTOPILOT.REPORTING_STREAM 			= Path.resolve(PATHS.SCRIPTS, "autopilot", "reportWithStreams.bat");
BATCHES.AUTOPILOT.TEST             			= {};
BATCHES.AUTOPILOT.TEST.ALL         			= Path.resolve(PATHS.SCRIPTS, "autopilot", "runAllTests.bat");
BATCHES.AUTOPILOT.TEST.SINGLE      			= Path.resolve(PATHS.SCRIPTS, "autopilot", "runAllTestsForComponent.bat");

BATCHES.AUTOPILOT.VIEWVERIFICATION 			= {};
BATCHES.AUTOPILOT.VIEWVERIFICATION.ALL    	= Path.resolve(PATHS.SCRIPTS, "autopilot", "verifyAllDesigns.bat");
BATCHES.AUTOPILOT.VIEWVERIFICATION.SINGLE 	= Path.resolve(PATHS.SCRIPTS, "autopilot", "verifyDesign.bat");

BATCHES.CLUSTERING.SIMULATION          		= {};
BATCHES.CLUSTERING.SIMULATION.GENERATE 		= Path.resolve(PATHS.SCRIPTS, "clustering", "generateClusterer.bat");
BATCHES.CLUSTERING.SIMULATION.EXECUTE  		= Path.resolve(PATHS.SCRIPTS, "clustering", "runClusterer.bat");
BATCHES.CLUSTERING.VISUALIZATION       		= Path.resolve(PATHS.SCRIPTS, "clustering", "visualize.bat");
BATCHES.CLUSTERING.REPORTING           		= Path.resolve(PATHS.SCRIPTS, "clustering", "report.bat");
BATCHES.CLUSTERING.REPORTING_STREAM    		= Path.resolve(PATHS.SCRIPTS, "clustering", "reportWithStreams.bat");
BATCHES.CLUSTERING.TEST                		= Path.resolve(PATHS.SCRIPTS, "clustering", "");

BATCHES.PUMP.VIEWVERIFICATION = {};
BATCHES.PUMP.VIEWVERIFICATION.ALL    = Path.resolve(PATHS.SCRIPTS, "pump", "verifyAllDesigns.bat");
BATCHES.PUMP.VIEWVERIFICATION.SINGLE = Path.resolve(PATHS.SCRIPTS, "pump", "verifyDesign.bat");
BATCHES.PUMP.VISUALIZATION       = Path.resolve(PATHS.SCRIPTS, "pump", "visualize.bat");
BATCHES.PUMP.REPORTING           		= Path.resolve(PATHS.SCRIPTS, "pump", "report.bat");

BATCHES.PACMAN.PLAY    		 		= {};
BATCHES.PACMAN.EMAM2WASM_GEN 		= Path.resolve(PATHS.SCRIPTS, "pacman", "emam2wasmGen.bat");
BATCHES.PACMAN.REPORTING        	= Path.resolve(PATHS.SCRIPTS, "pacman", "report.bat");
BATCHES.PACMAN.SIMULATION			= Path.resolve(PATHS.SCRIPTS, "pacman", "simulate.bat");
BATCHES.PACMAN.VISUALIZATION       		= Path.resolve(PATHS.SCRIPTS, "pacman", "visualize.bat");

BATCHES.INTERSECTION.SIMULATION				= Path.resolve(PATHS.SCRIPTS, "intersection","simulation.sh");
BATCHES.INTERSECTION.VISUALIZATION			= Path.resolve(PATHS.SCRIPTS, "intersection","visualize.sh");

BATCHES.CLASSIFIER.SIMULATION          		= {};
BATCHES.CLASSIFIER.SIMULATION.BUILD 		= Path.resolve(PATHS.SCRIPTS, "classifier", "build.sh");
BATCHES.CLASSIFIER.SIMULATION.EXECUTE 	= Path.resolve(PATHS.SCRIPTS, "classifier", "execute.sh");
BATCHES.CLASSIFIER.SIMULATION.REBUILD   = Path.resolve(PATHS.SCRIPTS, "classifier", "rebuild.sh");
BATCHES.CLASSIFIER.VISUALIZATION			= Path.resolve(PATHS.SCRIPTS, "classifier","visualize.sh");


/*
 * OPTIONS
 */
const OPTIONS = {};

OPTIONS.STATIC = {
	setHeaders: function(res, path, stat) {
		if(path.endsWith(".wasm")) {
			res.set("Content-Type", "application/wasm");
		}
	}
};

/*
 * PORTS
 */
const PORTS = {};

PORTS.SHARED = findPort();

/*
 * URLS
 */
const URLS = {};

URLS.SHARED               = Util.format("http://localhost:%d", PORTS.SHARED);
URLS.AUTOPILOT            = {};
URLS.AUTOPILOT.SIMULATION = "http://localhost:8080";
URLS.AUTOPILOT.DISTR_SIM  = "http://localhost/visualization";

/*
 * ARGUMENTS
 */
const ARGUMENTS = {};

/*
 * EXPORTS
 */
module.exports = {
	PATHS: PATHS,
	EXECUTABLES: EXECUTABLES,
	JARS: JARS,
	BATCHES: BATCHES,
	OPTIONS: OPTIONS,
	PORTS: PORTS,
	URLS: URLS,
	ARGUMENTS: ARGUMENTS
};
