/* (c) https://github.com/MontiCore/monticore */
/* caption: OCL; extensions: ocl */
define(function(require, exports, module) {
    var TaggingHighlightRules = require("./tagging_highlight_rules").TaggingHighlightRules;
    var Mode = require("plugins/se.rwth.api.language/modes/language");

    exports.Mode = Mode("Tagging", TaggingHighlightRules);
});
