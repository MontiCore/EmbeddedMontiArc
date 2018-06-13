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
		
		
		function onNFPResponse() {
			UICustom.done(messageIndex);
		}

		function onUpdated1() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Generating NFPVerification Test1 - This might take a moment...");
			var tab = TabManager.focussedTab;
			var path = tab.path;
            window.fetch("/services/" + project + "/test1", {
                "headers": { "Content-Type": "application/json" },
                "method": "post",
                "body": JSON.stringify({ name: path })
            }).then(onNFPResponse);
        }

		function onUpdated2() {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Generating NFPVerification Test2 - This might take a moment...");
			var tab = TabManager.focussedTab;
			var path = tab.path;
            window.fetch("/services/" + project + "/test2", {
                "headers": { "Content-Type": "application/json" },
                "method": "post",
                "body": JSON.stringify({ name: path })
            }).then(onNFPResponse);
        }
		
		function onReadFile(error, content) {
		    if(error) console.error(error);
		    else ModelsUpdater.update(onSingleUpdated);//handleContent(content); 
        }
		
		function handleTab1(tab) {
		    var path = tab.path;

		    if(path.endsWith(".ema")) ModelsUpdater.update(onUpdated1);
		    else window.alert("You have to open a ema in order to execute the test!");
        }
		
		function handleTab2(tab) {
		    var path = tab.path;

		    if(path.endsWith(".ema")) ModelsUpdater.update(onUpdated2);
		    else window.alert("You have to open a ema in order to execute the test!");
        }
		
		function onClick1() {
			var tab = TabManager.focussedTab;

            if(tab) handleTab1(tab);
			
		}

		function onClick2() {
            var tab = TabManager.focussedTab;

            if(tab) handleTab2(tab);
		}
		
        function onLoad() {
            if(loaded) {
                return false;
            } else {
                var parent = Layout.getElement("barTools");

                var nfpLabel1 = new UI.label({
                    "class": "icon-minus",
                    "height": 14,
                    "width": 22,
                    "tooltip": "Execute Test with rule 1",
                    "visible": true
                });

				var nfpLabel2 = new UI.label({
                    "class": "icon-plus",
                    "height": 14,
                    "width": 22,
                    "tooltip": "Execute Test with rule 2",
                    "visible": true
                });
				
				
				nfpLabel1.addEventListener("click", onClick1);
				nfpLabel2.addEventListener("click", onClick2);
                UI.insertByIndex(parent, nfpLabel1, 0, plugin);
				UI.insertByIndex(parent, nfpLabel2, 0, plugin);
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