/* (c) https://github.com/MontiCore/monticore */
/**
 * This file defines the Highlighting Rules of the
 * EmbeddedMontiView language.
 */
define(function(require, exports, module) {
    var keywords = (
        "package|component|atomic|view|extends|port|ports|(c)|in|out|timing|" +
        "instant|delayed|untimed|casualsync|sync|instance|connect|effect|autoinstantiate|" +
        "on|off"
    );

    var buildInConstants = ("");

    var langClasses = ("");

    var HighlightRules = require("plugins/se.rwth.api.language/modes/language_highlight_rules");

    exports.EmbeddedMontiViewHighlightRules =
        HighlightRules("EmbeddedMontiView", keywords, buildInConstants, langClasses);
});
