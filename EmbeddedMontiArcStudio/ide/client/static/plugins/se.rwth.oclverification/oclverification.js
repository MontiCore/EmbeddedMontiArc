define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui", "layout", "ui.custom", "tabManager", "models.updater", "fs"];
    main.provides = ["oclverification"];
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
        var pluginInformation = { "oclverification": plugin };

        var loaded = false;
		var messageIndex = -1;
		
		
		function onOCLResponse() {
			UICustom.done(messageIndex);
		}

		function onUpdatedCD() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Visualizing CD - This might take a moment...");
			var path = TabManager.focussedTab.path;
            window.fetch("/services/" + project + "/visualizeCD", {
                "headers": { "Content-Type": "application/json" },
                "method": "post",
                "body": JSON.stringify({ path: path })
            }).then(onOCLResponse);
        }

		function onUpdatedOCL() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Checking OCL - This might take a moment...");
			var tab = TabManager.focussedTab;
			var path = tab.path;
            window.fetch("/services/" + project + "/checkOCL", {
                "headers": { "Content-Type": "application/json" },
                "method": "post",
                "body": JSON.stringify({ name: path })
            }).then(onOCLResponse);
        }
		
		function onReadFile(error, content) {
		    if(error) console.error(error);
		    else ModelsUpdater.update(onSingleUpdated);
        }
		
		function handleTabCD(tab) {
		    var path = tab.path;

		    if(path.endsWith(".cd")) ModelsUpdater.update(onUpdatedCD);
		    else window.alert("You have to open a cd file in order to visualize it!");
        }
		
		function handleTabOCL(tab) {
		    var path = tab.path;

		    if(path.endsWith(".ocl")) ModelsUpdater.update(onUpdatedOCL);
		    else window.alert("You have to open an ocl file in order to check it!");
        }
		
		function onClickCD() {
			var tab = TabManager.focussedTab;

            if(tab) handleTabCD(tab);
			
		}

		function onClickOCL() {
            var tab = TabManager.focussedTab;

            if(tab) handleTab2(tab);
		}
		
        function onLoad() {
            if(loaded) {
                return false;
            } else {
                var parent = Layout.getElement("barTools");

                var labelCD = new UI.label({
                    "class": "icon-eye",
                    "height": 14,
                    "width": 22,
                    "tooltip": "Visualize CD",
                    "visible": true
                });

				var labelOCL = new UI.label({
                    "class": "icon-tick",
                    "height": 14,
                    "width": 22,
                    "tooltip": "Check OCL",
                    "visible": true
                });
				
				
				labelCD.addEventListener("click", onClickCD);
				labelOCL.addEventListener("click", onClickOCL);
                UI.insertByIndex(parent, labelCD, 0, plugin);
				UI.insertByIndex(parent, labelOCL, 0, plugin);
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