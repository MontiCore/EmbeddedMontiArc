/* (c) https://github.com/MontiCore/monticore */
/**
 * This file defines the Highlighting Rules of the
 * EmbeddedMontiArcDL language.
 */
define(function(require, exports, module) {
    var keywords = (
        "package|import|component|instance|implements|autoconnect|autoinstantiate|architecture|def|implementation"
    );

    var buildInConstants = ("");

    var langClasses = (
        "port|ports|connect|in|out|type|off|on|input|output"
    );

    var HighlightRules = require("plugins/se.rwth.api.language/modes/language_highlight_rules");

    exports.EMADLHighlightRules =
        HighlightRules("EmbeddedMontiArcDL", keywords, buildInConstants, langClasses);
});
