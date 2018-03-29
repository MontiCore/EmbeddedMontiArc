/* caption: MaCoCoVIZ; extensions: viz */
define(function(require, exports, module) {
    var MaCoCoVIZHighlightRules = require("./macocoviz_highlight_rules").MaCoCoVIZHighlightRules;
    var Mode = require("plugins/se.rwth.api.language/modes/language");

    exports.Mode = Mode("MaCoCoVIZ", MaCoCoVIZHighlightRules);
});