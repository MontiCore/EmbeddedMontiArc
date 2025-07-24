/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui", "layout", "ui.custom", "models.updater"];
    main.provides = ["reportWithStreams"];
    return main;

    function main(options, imports, register) {
        var Plugin = imports.Plugin;
        var UI = imports.ui;
        var Layout = imports.layout;
		var UICustom = imports["ui.custom"];
        var ModelsUpdater = imports["models.updater"];

        var plugin = new Plugin("SE RWTH", main.consumes);
        var pluginInformation = { "reportWithStreams": plugin };

        var loaded = false;
		var messageIndex = -1;
		
		
		function onReportResponse() {
			UICustom.done(messageIndex);
		}
		
		function onUpdated() {
            var project = localStorage.getItem("reponame").toLowerCase();

			messageIndex = UICustom.message("Generating Report with Streams - This can take up to 30 minutes...");
			
			window.fetch("/services/" + project + "/reportWS", {
				"method": "post"
			}).then(onReportResponse);
		}

        function onClick() {
            ModelsUpdater.update(onUpdated);
        }

        function onLoad() {
            if(loaded) {
                return false;
            } else {
                var parent = Layout.getElement("barTools");

                var exmarkLabel = new UI.label({
                    "class": "icon-exmark",
                    "height": 14,
                    "width": 22,
                    "tooltip": "Report With Streams",
                    "visible": true
                });

                exmarkLabel.addEventListener("click", onClick);
                UI.insertByIndex(parent, exmarkLabel, 0, plugin);
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
