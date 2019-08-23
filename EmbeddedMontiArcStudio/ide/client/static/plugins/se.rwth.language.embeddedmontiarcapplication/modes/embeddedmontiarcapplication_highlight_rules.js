/* (c) https://github.com/MontiCore/monticore */
/**
 * This file defines the Highlighting Rules of the
 * EmbeddedMontiArcApplication language.
 */
define(function(require, exports, module) {
    var keywords = (
        "package|import|component|instance|implements|autoconnect|autoinstantiate|implementation|set|to|load|texture|" +
        "from|unload|position|add|stage|create|sprite|remove|delay|hide|all|stages|delete|window|size"
    );

    var buildInConstants = ("visible|invisible|active|inactive");

    var langClasses = (
        "port|ports|connect|in|out|type|off|on|Application"
    );

    var HighlightRules = require("plugins/se.rwth.api.language/modes/language_highlight_rules");

    exports.EMAAPLHighlightRules =
        HighlightRules("OCL", keywords, buildInConstants, langClasses);
});
