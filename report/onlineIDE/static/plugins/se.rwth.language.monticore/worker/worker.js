define(function(require, exports, module) {
    var Handler = require("plugins/se.rwth.api.language/worker/worker");

    //5.0.0
    module.exports = Handler("MontiCore", 5 * 10000 + 0 * 1000 + 0 * 100 + 0);
});