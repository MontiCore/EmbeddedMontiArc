/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    return function(caption, version) {
        var captionLowerCase = caption.toLowerCase();
        var pluginPath = "plugins/se.rwth.language." + captionLowerCase;
        var staticPath = "/static/" + pluginPath;

        var baseHandler = require("plugins/c9.ide.language/base_handler");
        var handler = Object.create(baseHandler);
        var emitter = null;

        var Module = {};

        var callbacks = [];

        function toAbsolutePath(relativePath, callback) {
            var eventName = "toAbsolutePath:" + relativePath;

            emitter.once(eventName, callback);
            emitter.emit("toAbsolutePath", relativePath);
        }

        function importScriptsEx(relativePath, callback) {
            function onAbsolutePath(absolutePath) {
                callback = callback || function() {};
                importScripts(absolutePath);
                callback();
            }

            toAbsolutePath(relativePath, onAbsolutePath);
        }

        function instantiateWasm(imports, callback) {
            console.log(imports);
            function onInstantiate(instance) {
                callback(instance);
            }

            function onError(error) {
                console.error("An error occurred while instantiating the WebAssembly module:");
                console.error(error);
            }

            function onAbsolutePath(absolutePath) {
                instantiateCachedURL(version, absolutePath, imports).then(onInstantiate).catch(onError);
            }

            toAbsolutePath(staticPath + "/gen/" + captionLowerCase + ".wasm", onAbsolutePath);
            return {};
        }

        function executeCallbacks(docValue) {
            var length = callbacks.length;

            for(var i = 0; i < length; i++) {
                handleParse(docValue, callbacks[i]);
            }
        }


        function onModuleLoaded() {
            Module = self[caption](BaseModule);
        }

        function onBaseModuleLoaded() {
            BaseModule.onRuntimeInitialized = onRuntimeInitialized;
            BaseModule.instantiateWasm = instantiateWasm;
        }

        function onRuntimeInitialized() {
            var doc = handler.doc;
            var docValue = doc.getValue();

            emitter.emit("onRuntimeInitialized");
            executeCallbacks(docValue);
        }


        function init(callback) {
            emitter = handler.getEmitter();

            callback(null);
            importScriptsEx("/static/plugins/se.rwth.api.webassembly/lib/wasm-utils.js");
            importScriptsEx("/static/plugins/se.rwth.api.webassembly/module.js", onBaseModuleLoaded);
            importScriptsEx(staticPath + "/gen/" + captionLowerCase + ".js", onModuleLoaded);
        }

        function handlesLanguage(language) {
            return language === pluginPath + "/modes/" + captionLowerCase;
        }

        function parse(docValue, options, callback) {
            if(Module.parse) handleParse(docValue, callback);
            else callbacks.push(callback);
        }


        function replaceCommentMatch(match) {
            var length = match.length;
            var modMatch = "";

            for (var i = 0; i < length; i++) {
                modMatch += match[i] === ' ' || match[i] === '\n' ? match[i] : ' ';
            }

            return modMatch;
        }

        function removeComments(docValue) {
            return docValue.replace(/((['"])(?:(?!\2|\\).|\\.)*\2)|\/\/[^\n]*|\/\*(?:[^*]|\*(?!\/))*\*\//g, replaceCommentMatch);
        }

        function replaceAt(string, index, replacement) {
            return string.substr(0, index) + replacement + string.substr(index + replacement.length);
        }

        function doRemoveImplementation(docValue, index) {
            var bracketCounter = 0;
            var length = docValue.length;
            var inBody = false;

            for (var i = index; i < length; i++) {
                if (docValue[i] === '{') { bracketCounter++; inBody = true; }
                else if (docValue[i] === '}') bracketCounter--;

                docValue = replaceAt(docValue, i, docValue[i] !== '\n' ? ' ' : '\n');

                if (inBody && bracketCounter === 0) return docValue;
            }

            return docValue;
        }

        function removeImplementation(docValue, initial) {
            var start = docValue.indexOf("implementation", initial || 0);
            var middle = start === -1 ? -1 : docValue.indexOf("Math", start);

            if (middle === -1) return docValue;
            else return removeImplementation(doRemoveImplementation(docValue, start), start);
        }

        function removeNoise(docValue) {
            return removeImplementation(removeComments(docValue));
        }


        function handleParse(docValue, callback) {
            if (caption === "EmbeddedMontiArcMath") docValue = removeNoise(docValue);

            emitter.emit("onParse");

            console.debug("Parse");
            console.time("Parse");
            Module.parse(docValue);
            console.timeEnd("Parse");

            emitter.emit("onParsed");
            callback();
        }

        function analyze(docValue, ast, options, callback) {
            handleAnalyze(callback);
        }

        function handleAnalyze(callback) {
            var analyzes = Module.analyze();

            console.debug("Analyze:");
            console.debug(analyzes);

            callback(null, analyzes);
        }

        function outline(doc, ast, callback) {
            handleOutline(callback);
        }

        function handleOutline(callback) {
            var outline = { items: Module.outline() };

            console.debug("Outline:");
            console.debug(outline);

            callback(null, outline);
        }

        handler.init = init;
        handler.handlesLanguage = handlesLanguage;
        handler.parse = parse;
        handler.analyze = analyze;
        handler.outline = outline;

        return handler;
    };
});
