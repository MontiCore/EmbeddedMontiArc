/* (c) https://github.com/MontiCore/monticore */
/**
 * This file defines the Highlighting Rules of the
 * OCL language.
 */
define(function(require, exports, module) {
    var keywords = (
        "package|for|stream"
    );

    var buildInConstants = ("");

    var langClasses = ("tick");

    var HighlightRules = require("plugins/se.rwth.api.language/modes/language_highlight_rules");

    exports.OCLHighlightRules =
        HighlightRules("StreamUnits", keywords, buildInConstants, langClasses);
});
