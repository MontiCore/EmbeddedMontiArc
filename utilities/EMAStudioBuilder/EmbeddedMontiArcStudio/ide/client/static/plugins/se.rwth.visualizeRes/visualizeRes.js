/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui", "layout", "ui.custom", "models.updater"];
    main.provides = ["visualizeRes"];
    return main;

    function main(options, imports, register) {
        var Plugin = imports.Plugin;
        var UI = imports.ui;
        var Layout = imports.layout;
		var UICustom = imports["ui.custom"];
		var ModelsUpdater = imports["models.updater"];

        var plugin = new Plugin("SE RWTH", main.consumes);
        var pluginInformation = { "visualizeRes": plugin };

        var loaded = false;
		var messageIndex = -1;
		
		
		function onVisualizeResponse() {
			UICustom.done(messageIndex);
		}

		function onUpdated1() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Generating Visualization with resource interface - This might take a moment...");

            window.fetch("/services/" + project + "/visualizeRes1", {
                "method": "post"
            }).then(onVisualizeResponse);
        }

		function onUpdated2() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Generating Visualization with resource interface - This might take a moment...");

            window.fetch("/services/" + project + "/visualizeRes2", {
                "method": "post"
            }).then(onVisualizeResponse);
        }
		
		function onClick1() {
            ModelsUpdater.update(onUpdated1);
		}
		
		function onClick2() {
            ModelsUpdater.update(onUpdated2);
		}

        function onLoad() {
            if(loaded) {
                return false;
            } else {
                var parent = Layout.getElement("barTools");

                var resLabel1 = new UI.label({
                    "class": "icon-res",
                    "height": 14,
                    "width": 22,
                    "tooltip": "PIDError Visualization with Resources",
                    "visible": true
                });

				var resLabel2 = new UI.label({
                    "class": "icon-res",
                    "height": 14,
                    "width": 22,
                    "tooltip": "EngineAndBrakes Visualization with Resources",
                    "visible": true
                });

				resLabel1.addEventListener("click", onClick1);
                resLabel2.addEventListener("click", onClick2);
				
				UI.insertByIndex(parent, resLabel1, 0, plugin);
				UI.insertByIndex(parent, resLabel2, 0, plugin);
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
