/* (c) https://github.com/MontiCore/monticore */
/* caption: CNNTrainLang; extensions: cnnt */
define(function(require, exports, module) {
    var CNNTHighlightRules = require("./cnntrainlang_highlight_rules").CNNTHighlightRules;
    var Mode = require("plugins/se.rwth.api.language/modes/language");

    exports.Mode = Mode("CNNTrainLang", CNNTHighlightRules);
});
