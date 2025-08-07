/* (c) https://github.com/MontiCore/monticore */
/* caption: EmbeddedMontiView; extensions: emv */
define(function(require, exports, module) {
    var EmbeddedMontiViewHighlightRules = require("./embeddedmontiview_highlight_rules").EmbeddedMontiViewHighlightRules;
    var Mode = require("plugins/se.rwth.api.language/modes/language");

    exports.Mode = Mode("EmbeddedMontiView", EmbeddedMontiViewHighlightRules);
});
