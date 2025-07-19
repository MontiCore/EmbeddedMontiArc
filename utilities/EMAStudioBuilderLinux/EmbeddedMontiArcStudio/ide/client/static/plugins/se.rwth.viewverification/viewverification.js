/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui", "layout", "ui.custom", "tabManager", "models.updater", "fs"];
    main.provides = ["viewverification"];
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
        var pluginInformation = { "viewverification": plugin };

        var loaded = false;
		var messageIndex = -1;
		var collection = "";
		var streamName = "";


		function openPanel() {
		    var panes = TabManager.getPanes();
		    var pane = panes[0];
		    var options = {};

		    options.pane = pane.vsplit(true);
		    options.value = "http://localhost:8081/viewverification.html";
		    options.editorType = "urlview";
		    options.active = false;

		    TabManager.open(options, onTabOpen);
        }

        function handleTabOpen(tab) {
		    var document = tab.document;
		    var session = document.getSession();
		    var iframe = session.iframe;

		    function onLoad() {
                iframe.contentWindow.writeResults(collection);
            }

		    tab.title = "Verification Results";
		    iframe.style.marginTop = "1px";
		    iframe.style.height = "calc(100% - 1px)";
		    iframe.onload = onLoad;
        }

        function onTabOpen(error, tab) {
		    if(error) console.error("An error occurred while opening the tab:", error);
		    else handleTabOpen(tab);
        }

        function onSingleVerificationResponse(response) {
                UICustom.done(messageIndex);
        }
		
		function onVerificationAllResponse(response) {
                UICustom.done(messageIndex);
		}

        function onSingleUpdated() {
            var project = localStorage.getItem("reponame").toLowerCase();

            if(project === "clustering") return window.alert("Verification for Clustering is currently not setup!");

            messageIndex = UICustom.message("Verifying Design - This might take a minute...");
			var tab = TabManager.focussedTab;
			var path = tab.path;
            window.fetch("/services/" + project + "/viewverification/single", {
                "headers": { "Content-Type": "application/json" },
                "method": "post",
                "body": JSON.stringify({ name: path })
            }).then(onSingleVerificationResponse);
        }

		function onAllUpdated() {
            var project = localStorage.getItem("reponame").toLowerCase();

            if(project === "clustering") return window.alert("Verification for Clustering is currently not setup!");

            messageIndex = UICustom.message("Verifying All Designs - This can take up to 30 minutes...");

            window.fetch("/services/" + project + "/viewverification/all", {
                "method": "post"
            }).then(onVerificationAllResponse);
        }

        function formatStreamMatch(streamMatch) {
		    streamMatch = streamMatch.trim();

		    return streamMatch.charAt(0).toLowerCase() + streamMatch.slice(1);
        }

    /*    function handleMatches(packname, streamMatch) {
		    streamName = packname + '.' + formatStreamMatch(streamMatch);

		    ModelsUpdater.update(onSingleUpdated);
        }

        function handleContent(content) {
		    var packageMatches = content.match(/package (.+?);/);
		    var streamMatches = content.match(/stream .+? for (.+?){/);

		    if(packageMatches && streamMatches) handleMatches(packageMatches[1], streamMatches[1]);
        } */

        function onReadFile(error, content) {
		    if(error) console.error(error);
		    else ModelsUpdater.update(onSingleUpdated);//handleContent(content); 
        }

        function handleTab(tab) {
		    var path = tab.path;

		    if(path.endsWith(".emv")) ModelsUpdater.update(onSingleUpdated); //FileSystem.readFile(path, onReadFile);
		    else window.alert("You have to open a view in order to execute the verification!");
        }

        function onSingleClick() {
            var tab = TabManager.focussedTab;

            if(tab) handleTab(tab);
        }
		
		function onAllClick() {
            ModelsUpdater.update(onAllUpdated);
		}

        function onLoad() {
            if(loaded) {
                return false;
            } else {
                var parent = Layout.getElement("barTools");

                var verifyAllLabel = new UI.label({
                    "class": "icon-doubletick",
                    "height": 14,
                    "width": 22,
					"tooltip": "Verify All Designs",
                    "visible": true
                });

                var verifySingleLabel = new UI.label({
                    "class": "icon-tick",
                    "height": 14,
                    "width": 22,
                    "tooltip": "Verify Design",
                    "visible": true
                });

                verifyAllLabel.addEventListener("click", onAllClick);
                verifySingleLabel.addEventListener("click", onSingleClick);
                UI.insertByIndex(parent, verifyAllLabel, 0, plugin);
                UI.insertByIndex(parent, verifySingleLabel, 0, plugin);
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
