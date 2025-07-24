/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui", "layout", "ui.custom", "tabManager", "models.updater"];
    main.provides = ["stop"];
    return main;

    function main(options, imports, register) {
        var Plugin = imports.Plugin;
        var UI = imports.ui;
        var Layout = imports.layout;
		var UICustom = imports["ui.custom"];
		var TabManager = imports.tabManager;
		var ModelsUpdater = imports["models.updater"];

        var plugin = new Plugin("SE RWTH", main.consumes);
        var pluginInformation = { "stop": plugin };

        var loaded = false;
		var messageIndex = -1;

		function onStopResponse() {
			UICustom.done(messageIndex);
		}

		function onClick() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Stopping Simulation");

            window.fetch("/services/" + project + "/stop", {
                "method": "post"
            }).then(onStopResponse);
		}

        function onLoad() {
            var project = localStorage.getItem("reponame").toLowerCase();

            if(loaded) {
                return false;
            } else {
                var parent = Layout.getElement("barTools");

                if(project == "intersection") {
                  var backDivider = new UI.divider({
                      "class": "c9-divider-double menudivider"
                  });

                  var playLabel = new UI.label({
                      "class": "icon-stop",
                      "height": 14,
                      "width": 22,
                      "tooltip": "Stop simulator",
                      "visible": true
                  });

				          playLabel.addEventListener("click", onClick);
                  UI.insertByIndex(parent, playLabel, 0, plugin);
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
