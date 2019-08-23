/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui", "layout", "ui.custom", "tabManager", "models.updater", "fs"];
    main.provides = ["test"];
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
        var pluginInformation = { "test": plugin };

        var loaded = false;
		var messageIndex = -1;
		var collection = "";
		var streamName = "";


		function openPanel() {
		    var panes = TabManager.getPanes();
		    var pane = panes[0];
		    var options = {};
			var url = location.protocol + "//" + location.host + "/test.html";

		    options.pane = pane.vsplit(true);
		    options.value = url;
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

		    tab.title = "Test Results";
		    iframe.style.marginTop = "1px";
		    iframe.style.height = "calc(100% - 1px)";
		    iframe.onload = onLoad;
        }

        function onTabOpen(error, tab) {
		    if(error) console.error("An error occurred while opening the tab:", error);
		    else handleTabOpen(tab);
        }

        function onSingleTestResponse(response) {
            function onThen(results) {
                collection = results;

                UICustom.done(messageIndex);
                openPanel();
            }

            response.text().then(onThen);
        }
		
		function onTestAllResponse(response) {
		    function onThen(results) {
		        collection = results;

                UICustom.done(messageIndex);
                openPanel();
            }

            response.text().then(onThen);
		}

        function onSingleUpdated() {
            var project = localStorage.getItem("reponame").toLowerCase();

            if(project === "clustering") return window.alert("Testing for Clustering is currently not setup!");

            messageIndex = UICustom.message("Running Test - This might take a minute...");

            window.fetch("/services/" + project + "/test/single", {
                "headers": { "Content-Type": "application/json" },
                "method": "post",
                "body": JSON.stringify({ streamName: streamName })
            }).then(onSingleTestResponse);
        }

		function onAllUpdated() {
            var project = localStorage.getItem("reponame").toLowerCase();

            if(project === "clustering") return window.alert("Testing for Clustering is currently not setup!");

            messageIndex = UICustom.message("Running All Tests - This can take up to 30 minutes...");

            window.fetch("/services/" + project + "/test/all", {
                "method": "post"
            }).then(onTestAllResponse);
        }

        function formatStreamMatch(streamMatch) {
		    streamMatch = streamMatch.trim();

		    return streamMatch.charAt(0).toLowerCase() + streamMatch.slice(1);
        }

        function handleMatches(packname, streamMatch) {
		    streamName = packname + '.' + formatStreamMatch(streamMatch);

		    ModelsUpdater.update(onSingleUpdated);
        }

        function handleContent(content) {
		    var packageMatches = content.match(/package (.+?);/);
		    var streamMatches = content.match(/stream .+? for (.+?){/);

		    if(packageMatches && streamMatches) handleMatches(packageMatches[1], streamMatches[1]);
        }

        function onReadFile(error, content) {
		    if(error) console.error(error);
		    else handleContent(content);
        }

        function handleTab(tab) {
		    var path = tab.path;

		    if(path.endsWith(".stream")) FileSystem.readFile(path, onReadFile);
		    else window.alert("You have to open a stream in order to execute the test!");
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

                var testAllLabel = new UI.label({
                    "class": "icon-test",
                    "height": 14,
                    "width": 22,
					"tooltip": "Run All Tests",
                    "visible": true
                });

                var testSingleLabel = new UI.label({
                    "class": "icon-test",
                    "height": 14,
                    "width": 22,
                    "tooltip": "Run Test",
                    "visible": true
                });

                testAllLabel.addEventListener("click", onAllClick);
                testSingleLabel.addEventListener("click", onSingleClick);
                UI.insertByIndex(parent, testAllLabel, 0, plugin);
                UI.insertByIndex(parent, testSingleLabel, 0, plugin);
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
