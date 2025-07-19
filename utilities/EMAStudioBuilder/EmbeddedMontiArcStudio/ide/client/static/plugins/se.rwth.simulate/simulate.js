/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui", "layout", "ui.custom", "tabManager", "models.updater"];
    main.provides = ["simulate"];
    return main;

    function main(options, imports, register) {
        var Plugin = imports.Plugin;
        var UI = imports.ui;
        var Layout = imports.layout;
		var UICustom = imports["ui.custom"];
		var TabManager = imports.tabManager;
		var ModelsUpdater = imports["models.updater"];

        var plugin = new Plugin("SE RWTH", main.consumes);
        var pluginInformation = { "simulate": plugin };

        var loaded = false;
		var messageIndex = -1;

		function onSimulateResponse() {
			UICustom.done(messageIndex);
		}

		function onUpdated() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Starting Simulation - This might take a minute...");

            window.fetch("/services/" + project + "/simulate", {
                "method": "post"
            }).then(onSimulateResponse);
		}

        function onUpdatedDistr() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Starting Distributed Simulation - This might take a minute...");

            window.fetch("/services/" + project + "/simulate-distr", {
                "method": "post"
            }).then(onSimulateResponse);
		}

		function onClick() {
			ModelsUpdater.update(onUpdated);
		}

        function onClickDistr() {
			ModelsUpdater.update(onUpdatedDistr);
		}

        function onLoad() {
            var project = localStorage.getItem("reponame").toLowerCase();

            if(loaded) {
                return false;
            } else {
                var parent = Layout.getElement("barTools");

                var backDivider = new UI.divider({
                    "class": "c9-divider-double menudivider"
                });

                var playLabel = new UI.label({
                    "class": "icon-play",
                    "height": 14,
                    "width": 22,
                    "tooltip": "Execute Model",
                    "visible": true
                });

				playLabel.addEventListener("click", onClick);
                UI.insertByIndex(parent, playLabel, 0, plugin);
                UI.insertByIndex(parent, backDivider, 10, plugin);

                if(project == "autopilot") {
                    var playDistrLabel = new UI.label({
                        "class": "icon-play distr",
                        "height": 14,
                        "width": 22,
                        "tooltip": "Execute Model in Distributed Simulator",
                        "visible": true
                    });

                    playDistrLabel.addEventListener("click", onClickDistr);
                    UI.insertByIndex(parent, playDistrLabel, 0, plugin);
                    UI.insertByIndex(parent, backDivider, 10, plugin);
                }
            }
        }

        function onUnload() {
            loaded = false;
        }

        plugin.on("load", onLoad);
        plugin.on("unload", onUnload);
        register(null, pluginInformation);
    }
});
