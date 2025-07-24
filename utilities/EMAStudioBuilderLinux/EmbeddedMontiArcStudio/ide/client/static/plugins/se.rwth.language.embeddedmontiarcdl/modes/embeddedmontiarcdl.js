/* (c) https://github.com/MontiCore/monticore */
/* caption: EmbeddedMontiArcDL; extensions: emadl */
define(function(require, exports, module) {
    var EMADLHighlightRules = require("./embeddedmontiarcdl_highlight_rules").EMADLHighlightRules;
    var Mode = require("plugins/se.rwth.api.language/modes/language");

    exports.Mode = Mode("EmbeddedMontiArcDL", EMADLHighlightRules);
});
