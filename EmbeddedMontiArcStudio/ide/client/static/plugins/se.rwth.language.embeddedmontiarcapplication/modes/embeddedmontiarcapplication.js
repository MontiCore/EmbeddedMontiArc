/* (c) https://github.com/MontiCore/monticore */
/* caption: EmbeddedMontiArcApplication; extensions: emaapl */
define(function(require, exports, module) {
    var EMAAPLHighlightRules = require("./embeddedmontiarcapplication_highlight_rules").EMAAPLHighlightRules;
    var Mode = require("plugins/se.rwth.api.language/modes/language");

    exports.Mode = Mode("EmbeddedMontiArcApplication", EMAAPLHighlightRules);
});
