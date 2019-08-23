/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "fs", "tabManager"];
    main.provides = ["readme.opener"];
    return main;

    function main(options, imports, register) {
        var Plugin = imports.Plugin;
        var fs = imports.fs;
        var tabManager = imports.tabManager;

        var plugin = new Plugin("SE RWTH", main.consumes);
        var pluginInformation = { "readme.opener": plugin };

        function onPluginLoad(error) {
            if(error) console.error(error);
            else fs.exists("/README.md", onExists);
        }

        function onExists(exists) {
            if(exists) handleExists();
        }

        function handleExists() {
            var options = {};

            options.path = "/README.md";
            options.editorType = "preview";
            options.active = true;

            tabManager.open(options, onTabOpen);
        }

        function onTabOpen(error, tab) {
            if(error) console.error(error);
        }

        plugin.on("load", onPluginLoad);
        register(null, pluginInformation);
    }
});
