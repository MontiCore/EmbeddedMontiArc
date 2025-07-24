/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui", "layout", "ui.custom", "models.updater"];
    main.provides = ["play"];
    return main;

    function main(options, imports, register) {
        var Plugin = imports.Plugin;
        var UI = imports.ui;
        var Layout = imports.layout;
		var UICustom = imports["ui.custom"];
		var ModelsUpdater = imports["models.updater"];

        var plugin = new Plugin("SE RWTH", main.consumes);
        var pluginInformation = { "play": plugin };

        var loaded = false;
		var messageIndex = -1;
		
		
		function onPlayResponse() {
			UICustom.done(messageIndex);
		}

		function onUpdated() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Starting Pacman - This might take a few minutes...");

            window.fetch("/services/" + project + "/play", {
                "method": "post"
            }).then(onPlayResponse);
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
                    "class": "icon-pacman-play",
                    "height": 14,
                    "width": 22,
                    "tooltip": "Play Pacman",
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
