const { series, concurrent } = require("nps-utils");

function getLernaCommand(script, parallelized) {
    const target = script.startsWith("target");
    const scope = (target ? "--scope" : "--ignore") + " \"@elysium/ide\"";
    const config = (target ? "../" : "../../") + "configs/scripts";
    const parallel = parallelized ? " --parallel " : ' ';

    return "lerna exec --stream " + scope + parallel + "-- nps -c " + config + " --no-scripts " + script;
}

module.exports = {
    scripts: {
        "clean": {
            "default": {
                "script": series("node scripts/logo", concurrent({
                    "Extensions": {
                        "script": getLernaCommand("extension.clean", true),
                        "color": "greenBright"
                    },
                    "Target": {
                        "script": getLernaCommand("target.clean", false),
                        "color": "greenBright"
                    }
                })),
                "description": "Removes 'lib' directories and generated Theia files."
            },
            "all": {
                "script": series("node scripts/logo", concurrent({
                    "Root": {
                        "script": "rimraf .browser_modules && rimraf lerna-debug.log && rimraf yarn-error.log",
                        "color": "greenBright"
                    },
                    "Node Modules": {
                        "script": "lerna clean",
                        "color": "greenBright"
                    },
                    "Target": {
                        "script": getLernaCommand("target.clean.all", false),
                        "color": "greenBright"
                    }
                })),
                "description": "Everything that can be removed, will be removed."
            }
        },
        "compile": {
            "default": {
                "script": series("node scripts/logo", getLernaCommand("extension.compile", false)),
                "description": "Compiles the source files using the TypeScript compiler."
            },
            "clean": {
                "script": getLernaCommand("extension.compile.clean", true),
                "description": "Deletes the compiled files."
            }
        },
        "build": {
            "default": {
                "script": series("node scripts/logo", getLernaCommand("target.build", false)),
                "description": "Builds the EmbeddedMontiArcStudio target in production mode."
            },
            "development": {
                "script": series("node scripts/logo", getLernaCommand("target.build.development", false)),
                "description": "Builds the EmbeddedMontiArcStudio target in development mode."
            }
        },
        "docs": {
            "default": {
                "script": series("node scripts/logo", getLernaCommand("extension.docs", true)),
                "description": "Generates the TypeDoc documentation."
            },
            "clean": {
                "script": getLernaCommand("extension.docs.clean", true),
                "description": "Deletes the TypeDoc documentation."
            }
        },
        "package": {
            "default": {
                "script": series("node scripts/logo", getLernaCommand("target.package", false)),
                "description": "Creates the Electron application with installer."
            }
        },
        "default": {
            "script": series("node scripts/logo", getLernaCommand("target.start", false)),
            "description": "Locally host the IDE."
        },
        "extension": {
            "clean": {
                "script": concurrent({
                    "Libraries": {
                        "script": "nps -c ../../configs/scripts --no-scripts extension.compile.clean",
                        "color": "white"
                    },
                    "Documentations": {
                        "script": "nps -c ../../configs/scripts --no-scripts extension.docs.clean",
                        "color": "white"
                    }
                }),
                "hiddenFromHelp": true
            },
            "compile": {
                "default": {
                    "script": "tsc --project compile.tsconfig.json",
                    "hiddenFromHelp": true
                },
                "clean": {
                    "script": "rimraf lib",
                    "hiddenFromHelp": true
                }
            },
            "watch": {
                "script": "tsc --watch --project compile.tsconfig.json",
                "hiddenFromHelp": true
            },
            "docs": {
                "default": {
                    "script": "typedoc --tsconfig compile.tsconfig.json --options ../../configs/typedoc.json",
                    "hiddenFromHelp": true
                },
                "clean": {
                    "script": "rimraf docs",
                    "hiddenFromHelp": true
                }
            }
        },
        "target": {
            "clean": {
                "default": {
                    "script": "theia clean",
                    "hiddenFromHelp": true
                },
                "all": {
                    "script": "theia clean && rimraf dist",
                    "hiddenFromHelp": true
                }
            },
            "build": {
                "default": {
                    "script": "theia build --config configs/webpack.config.js",
                    "hiddenFromHelp": true
                },
                "development": {
                    "script": "theia build --config configs/webpack.config.js --mode development",
                    "hiddenFromHelp": true
                }
            },
            "package": {
                "script": "node scripts/package",
                "hiddenFromHelp": true
            },
            "start": {
                "script": "node scripts/start",
                "hiddenFromHelp": true
            }
        }
    }
};