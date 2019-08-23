/* (c) https://github.com/MontiCore/monticore */
/**
 * This file defines the Highlighting Rules of the
 * OCL language.
 */
define(function(require, exports, module) {
    var keywords = (
        "package|conforms|to|tags|tag|for|with|NameScope|ConnectorScope"
    );

    var buildInConstants = ("undefined");

    var langClasses = ("");

    var HighlightRules = require("plugins/se.rwth.api.language/modes/language_highlight_rules");

    exports.TaggingHighlightRules =
        HighlightRules("Tagging", keywords, buildInConstants, langClasses);
});
