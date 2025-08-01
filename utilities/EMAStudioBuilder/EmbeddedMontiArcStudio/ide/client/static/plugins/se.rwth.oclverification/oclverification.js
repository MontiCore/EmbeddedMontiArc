/* (c) https://github.com/MontiCore/monticore */
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
		var oclResults = "";
		var consoleTab;


		function openPanel() {
		    if(consoleTab) {
		        consoleTab.close();
		    }


		    var panes = TabManager.getPanes();
		    var pane = panes[0];
		    var options = {};
			var url = location.protocol + "//" + location.host + "/test.html";

		    options.pane = pane.vsplit(true);
		    options.value = url;
		    options.editorType = "urlview";
		    options.active = false;

		    consoleTab = TabManager.open(options, onTabOpen);
        }

        function handleTabOpen(tab) {
		    var document = tab.document;
		    var session = document.getSession();
		    var iframe = session.iframe;

		    function onLoad() {
                iframe.contentWindow.writeResults(oclResults);
            }

		    tab.title = "OCL correctness:";
		    iframe.style.marginTop = "1px";
		    iframe.style.height = "calc(100% - 1px)";
		    iframe.onload = onLoad;
        }

        function onTabOpen(error, tab) {
		    if(error) console.error("An error occurred while opening the tab:", error);
		    else handleTabOpen(tab);
        }
		
		function onCDResponse() {
			UICustom.done(messageIndex);
		}

		function onOCLResponse(response) {
            function onThen(results) {
                oclResults = "<pre>" + results + "</pre>";

                UICustom.done(messageIndex);
                openPanel();
            }

            response.text().then(onThen);
		}

		function onUpdatedCD() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Visualizing CD - This might take a moment...");
			var path = TabManager.focussedTab.path;
            window.fetch("/services/" + project + "/visualizeCD", {
                "headers": { "Content-Type": "application/json" },
                "method": "post",
                "body": JSON.stringify({ path: path })
            }).then(onCDResponse);
        }

		function onUpdatedOCL() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Checking OCL - This might take a moment...");
			var tab = TabManager.focussedTab;
			var path = tab.path;
            window.fetch("/services/" + project + "/checkOCL", {
                "headers": { "Content-Type": "application/json" },
                "method": "post",
                "body": JSON.stringify({ path: path })
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

            if(tab) handleTabOCL(tab);
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
