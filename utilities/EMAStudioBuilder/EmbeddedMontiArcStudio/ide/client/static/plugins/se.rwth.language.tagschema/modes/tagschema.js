/* (c) https://github.com/MontiCore/monticore */
/* caption: TagSchema; extensions: tagschema */
define(function(require, exports, module) {
    var TagSchemaHighlightRules = require("./tagschema_highlight_rules").TagSchemaHighlightRules;
    var Mode = require("plugins/se.rwth.api.language/modes/language");

    exports.Mode = Mode("TagSchema", TagSchemaHighlightRules);
});
