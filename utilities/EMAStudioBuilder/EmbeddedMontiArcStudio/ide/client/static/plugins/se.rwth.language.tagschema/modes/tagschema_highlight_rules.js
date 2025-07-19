/* (c) https://github.com/MontiCore/monticore */
/**
 * This file defines the Highlighting Rules of the
 * TagSchema language.
 */
define(function(require, exports, module) {
    var keywords = (
        "package|tagschema|Component|Port|Connector|ExpandedComponentInstance|for|private|tagtype"
    );

    var buildInConstants = ("undefined");

    var langClasses = ("Number|String|Boolean");

    var HighlightRules = require("plugins/se.rwth.api.language/modes/language_highlight_rules");

    exports.TagSchemaHighlightRules =
        HighlightRules("TagSchema", keywords, buildInConstants, langClasses);
});
