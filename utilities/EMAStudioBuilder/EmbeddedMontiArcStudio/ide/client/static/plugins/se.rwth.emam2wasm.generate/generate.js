/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui", "layout", "ui.custom", "tabManager", "models.updater"];
    main.provides = ["generate"];
    return main;

    function main(options, imports, register) {
        var Plugin = imports.Plugin;
        var UI = imports.ui;
        var Layout = imports.layout;
		var UICustom = imports["ui.custom"];
		var ModelsUpdater = imports["models.updater"];
		var TabManager = imports.tabManager;

        var plugin = new Plugin("SE RWTH", main.consumes);
        var pluginInformation = { "generate": plugin };

        var loaded = false;
		var messageIndex = -1;
		
		
		function onReportResponse() {
			UICustom.done(messageIndex);
		}

		function onUpdated() {
            var tab = TabManager.focussedTab.path;
            
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Generating Web Assembly - This might take a few minutes...");

            window.fetch("/services/" + project + "/emam2wasmGen", {
                "headers": { "Content-Type": "application/json" },
                "method": "post",
                "body": JSON.stringify({ tab: tab })
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
                    "class": "icon-generate",
                    "height": 14,
                    "width": 22,
                    "tooltip": "Generate Web Assembly",
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
