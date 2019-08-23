/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    main.consumes = ["Plugin", "ui.custom", "fs"];
    main.provides = ["models.updater"];
    return main;

    function main(options, imports, register) {
        var Plugin = imports.Plugin;
        var UICustom = imports["ui.custom"];
        var FileSystem = imports.fs;

        var plugin = new Plugin("SE RWTH", main.consumes);
        var pluginInformation = { "models.updater": plugin };

        var messageIndex = -1;

        var API = {
            update: update
        };


        function update(callback) {
            var project = localStorage.getItem("reponame").toLowerCase();

            messageIndex = UICustom.message("Storing Files...");

            function onResponse() {
                UICustom.done(messageIndex);
                callback();
            }

            function onFilesFormatted(error, collection) {
                if(error) return console.error(error);

                window.fetch("/services/" + project + "/update-models", {
                    "headers": { "Content-Type": "application/json" },
                    "method": "post",
                    "body": JSON.stringify(collection)
                }).then(onResponse);
            }

            function onCollectedFiles(error, files) {
                if(error) console.error(error);
                else formatFiles(files, onFilesFormatted);
            }

            collectFiles(onCollectedFiles);
        }

        function collectFiles(callback) {
            var stack = ['/'];

            doCollectFiles(stack, [], callback);
        }

        function doCollectFiles(stack, collection, callback) {
            var directory = stack.shift();

            function onDirectoryHandled(error) {
                if(error) callback(error);
                else doCollectFiles(stack, collection, callback);
            }

            function onReaddir(error, paths) {
                if(error) callback(error);
                else doHandleDirectory(paths, stack, collection, onDirectoryHandled);
            }

            if(directory) FileSystem.readdir(directory, onReaddir);
            else callback(null, collection);
        }

        function doHandleDirectory(stats, stack, collection, callback) {
            var stat = stats.shift();

            if(stat && !stat.fullPath.startsWith("/.c9") && !stat.fullPath.startsWith("/.home")) {
                if (stat && stat.mime === "folder") stack.push(stat.fullPath);
                else if (stat && stat.mime === "file") collection.push(stat.fullPath);
            }

            if(stat) doHandleDirectory(stats, stack, collection, callback);
            else callback(null);
        }

        function formatFiles(files, callback) {
            doFormatFiles(files, [], callback);
        }

        function doFormatFiles(files, collection, callback) {
            var file = files.shift();

            function onContentHandled(error) {
                if(error) callback(error);
                else doFormatFiles(files, collection, callback);
            }

            function onReadFile(error, content) {
                if(error) callback(error);
                else doHandleContent(file, content, collection, onContentHandled);
            }

            if(file) FileSystem.readFile(file, onReadFile);
            else callback(null, collection);
        }

        function doHandleContent(file, content, collection, callback) {
            var entry = {};

            entry.path = file;
            entry.content = content;

            collection.push(entry);
            callback(null);
        }


        register(null, pluginInformation);
        plugin.freezePublicAPI(API);
    }
});
