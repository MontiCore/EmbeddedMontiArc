/**
 * This file defines the Highlighting Rules of the
 * MontiCore language.
 */
define(function(require, exports, module) {
    var keywords = (
        "package|component|grammar|extends|import|options|start|fragment|comment|token|enum|external|interface|" +
        "astextends|abstract|implements|astimplements|init|EOF|MCA|concept|astrule|ast|symbol|scope|method"
    );

    var buildInConstants = (
        "follow|min|max|public|private|protected|final|static|throws"
    );

    var langClasses = ("");

    var HighlightRules = require("plugins/se.rwth.api.language/modes/language_highlight_rules");

    exports.MontiCoreHighlightRules =
        HighlightRules("MontiCore", keywords, buildInConstants, langClasses);
});