/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui", "layout", "ui.custom", "tabManager", "models.updater", "fs"];
    main.provides = ["nfpverification"];
    return main;

    function main(options, imports, register) {
        var Plugin = imports.Plugin;
        var UI = imports.ui;
        var Layout = imports.layout;
		var UICustom = imports["ui.custom"];
		var TabManager = imports.tabManager;
		var ModelsUpdater = imports["models.updater"];
		var FileSystem = imports.fs;
		
        var plugin = new Plugin("SE RWTH", main.consumes);
        var pluginInformation = { "nfpverification": plugin };

        var loaded = false;
		var messageIndex = -1;
		
		
		function onFinish() {
			UICustom.done(messageIndex);
		}
		
		function onNFPResponse() {
			ModelsUpdater.update(onFinish)
		}

		function onUpdated() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Generating NFPVerification Test - This might take a moment...");
			var tab = TabManager.focussedTab;
			var path = tab.path;
            window.fetch("/services/" + project + "/test", {
                "headers": { "Content-Type": "application/json" },
                "method": "post",
                "body": JSON.stringify({ name: path })
            }).then(onNFPResponse);
        }
		
		function onReadFile(error, content) {
		    if(error) console.error(error);
		    else ModelsUpdater.update(onSingleUpdated);//handleContent(content); 
        }
		
		function handleTab(tab) {
		    var path = tab.path;

		    if(path.endsWith(".ocl")) ModelsUpdater.update(onUpdated);
		    else window.alert("You have to open an ocl rule in order to execute the test!");
        }
		
		function onClick() {
			var tab = TabManager.focussedTab;

            if(tab) handleTab(tab);
			
		}

        function onLoad() {
            if(loaded) {
                return false;
            } else {
                var parent = Layout.getElement("barTools");

                var nfpLabel = new UI.label({
                    "class": "icon-minus",
                    "height": 14,
                    "width": 22,
                    "tooltip": "Execute Test",
                    "visible": true
                });
				
				
				nfpLabel.addEventListener("click", onClick);
                UI.insertByIndex(parent, nfpLabel, 0, plugin);
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
