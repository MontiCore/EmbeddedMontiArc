/* (c) https://github.com/MontiCore/monticore */
/**
 * This file defines the Highlighting Rules of the
 * CNNTrainLang language.
 */
define(function(require, exports, module) {
    var keywords = (
        "configuration"
    );

    var buildInConstants = ("true|false|fixed|step|exp|inv|poly|sigmoid|cpu|gpu");

    var langClasses = (
        "num_epoch|batch_size|load_checkpoint|normalize|optimizer|context|sgd|adam|rmsprop|adagrad|nag|adadelta|" +
        "learning_rate_minimum|learning_rate|weight_decay|learning_rate_decay|learning_rate_policy|rescale_grad|" +
        "clip_gradient|step_size|momentum|beta1|beta2|epsilon|gamma1|gamma2|centered|clip_weights|rho"
    );

    var HighlightRules = require("plugins/se.rwth.api.language/modes/language_highlight_rules");

    exports.CNNTHighlightRules =
        HighlightRules("CNNTrainLang", keywords, buildInConstants, langClasses);
});
