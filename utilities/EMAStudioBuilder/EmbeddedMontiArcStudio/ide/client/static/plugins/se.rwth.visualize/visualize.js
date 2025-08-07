/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui", "layout", "ui.custom", "models.updater"];
    main.provides = ["visualize"];
    return main;

    function main(options, imports, register) {
        var Plugin = imports.Plugin;
        var UI = imports.ui;
        var Layout = imports.layout;
		var UICustom = imports["ui.custom"];
		var ModelsUpdater = imports["models.updater"];

        var plugin = new Plugin("SE RWTH", main.consumes);
        var pluginInformation = { "visualize": plugin };

        var loaded = false;
		var messageIndex = -1;
		
		
		function onVisualizeResponse() {
			UICustom.done(messageIndex);
		}

		function onUpdated() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Generating Visualization - This might take a moment...");

            window.fetch("/services/" + project + "/visualize", {
                "method": "post"
            }).then(onVisualizeResponse);
        }

		function onClick() {
            ModelsUpdater.update(onUpdated);
		}

        function onLoad() {
            if(loaded) {
                return false;
            } else {
                var parent = Layout.getElement("barTools");

                var eyeLabel = new UI.label({
                    "class": "icon-eye",
                    "height": 14,
                    "width": 22,
                    "tooltip": "Generate Visualization",
                    "visible": true
                });

				eyeLabel.addEventListener("click", onClick);
                UI.insertByIndex(parent, eyeLabel, 0, plugin);
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
