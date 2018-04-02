/* caption: MontiCore; extensions: mc4 */
define(function(require, exports, module) {
    var MontiCoreHighlightRules = require("./monticore_highlight_rules").MontiCoreHighlightRules;
    var Mode = require("plugins/se.rwth.api.language/modes/language");

    exports.Mode = Mode("MontiCore", MontiCoreHighlightRules);
});