/* (c) https://github.com/MontiCore/monticore */
define(function(require, exports, module) {
    var Handler = require("plugins/se.rwth.api.language/worker/worker");

    //1.3.12-SNAPSHOT
    module.exports = Handler("CD", 1 * 10000 + 3 * 1000 + 12 * 100 + 5);
});
