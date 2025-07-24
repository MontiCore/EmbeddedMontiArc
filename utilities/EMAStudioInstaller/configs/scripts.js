const { series, concurrent } = require("nps-utils");

function getScopeFromScript(script) {
    if (script.startsWith("target")) return "--scope" + " \"@emastudio/emastudio\"";
    else if (script.startsWith("manager")) return "--scope" + " \"@emastudio/manager\"";
    else return "--ignore" + " \"@emastudio/+(emastudio|manager)\"";
}

function getConfigFromScript(script) {
    if (script.startsWith("target") || script.startsWith("manager")) return "../configs/scripts";
    else return "../../configs/scripts";
}

function getLernaCommand(script, parallelized) {
    const scope = getScopeFromScript(script);
    const config = getConfigFromScript(script);
    const parallel = parallelized ? " --parallel " : ' ';

    return "lerna exec --stream " + scope + parallel + "-- nps -c " + config + " --no-scripts " + script;
}

module.exports = {
    scripts: {
        "default": {
            "script": series("node scripts/logo", getLernaCommand("target.start", false)),
            "description": "Injects build and executes EmbeddedMontiArcStudio."
        },
        "clean": {
            "default": {
                "script": series("node scripts/logo", concurrent({
                    "Extensions": {
                        "script": getLernaCommand("extension.clean", true),
                        "color": "blueBright"
                    },
                    "Target": {
                        "script": getLernaCommand("target.clean", false),
                        "color": "blueBright"
                    }
                })),
                "description": "Removes 'lib' directories and generated Theia files."
            },
            "all": {
                "script": series("node scripts/logo", concurrent({
                    "Extensions": {
                        "script": getLernaCommand("extension.clean.all", true),
                        "color": "blueBright"
                    },
                    "Target": {
                        "script": getLernaCommand("target.clean.all", false),
                        "color": "blueBright"
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
                "script": series("node scripts/logo", "electron-builder --config configs/electron-builder.yml"),
                "description": "Creates the Electron application with installer."
            },
            "dry": {
                "script": series("node scripts/logo", "electron-builder --config configs/electron-builder.yml --dir"),
                "description": "Creates the Electron application without installer."
            }
        },
        "manage": {
            "script": series("node scripts/logo", getLernaCommand("manager.manage", false)),
            "description": "Launches the EmbeddedMontiArcStudio Installer Manager."
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
                    "script": "typedoc --tsconfig compile.tsconfig.json --options ../../ide/configs/typedoc.json",
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
                    "script": "theia clean && node scripts/clean",
                    "hiddenFromHelp": true
                }
            },
            "build": {
                "default": {
                    "script": series(
                        "theia build --config configs/webpack.config.js",
                        "node scripts/build"
                    ),
                    "hiddenFromHelp": true
                },
                "development": {
                    "script": series(
                        "theia build --config configs/webpack.config.js --mode development",
                        "node scripts/build"
                    ),
                    "hiddenFromHelp": true
                }
            },
            "start": {
                "script": "node scripts/start",
                "hiddenFromHelp": true
            }
        },
        "manager": {
            "manage": {
                "script": "electron .",
                "hiddenFromHelp": true
            }
        }
    }
};