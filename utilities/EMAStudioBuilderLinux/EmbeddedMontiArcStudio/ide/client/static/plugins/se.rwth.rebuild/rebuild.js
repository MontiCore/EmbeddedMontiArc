/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui", "layout", "ui.custom", "tabManager", "models.updater"];
    main.provides = ["rebuild"];
    return main;

    function main(options, imports, register) {
        var Plugin = imports.Plugin;
        var UI = imports.ui;
        var Layout = imports.layout;
		var UICustom = imports["ui.custom"];
		var TabManager = imports.tabManager;
		var ModelsUpdater = imports["models.updater"];

        var plugin = new Plugin("SE RWTH", main.consumes);
        var pluginInformation = { "rebuild": plugin };

        var loaded = false;
		var messageIndex = -1;

		function onRebuildResponse() {
			UICustom.done(messageIndex);
		}

		function onClick() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Rebuilding Project..");

            window.fetch("/services/" + project + "/rebuild", {
                "method": "post"
            }).then(onRebuildResponse);
		}

        function onLoad() {
            var project = localStorage.getItem("reponame").toLowerCase();

            if(loaded) {
                return false;
            } else {
                var parent = Layout.getElement("barTools");

                if(project == "classifier") {
                    var rebuildLabel = new UI.label({
                        "class": "icon-rebuild",
                        "height": 14,
                        "width": 22,
                        "tooltip": "Rebuild Project",
                        "visible": true
                    });
                    rebuildLabel.addEventListener("click", onClick);
                    UI.insertByIndex(parent, rebuildLabel, 0, plugin);
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
