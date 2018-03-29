/**
 * This file defines the Highlighting Rules of the
 * MaCoCoVIZ language.
 */
define(function(require, exports, module) {
    var keywords = (
        "vizualization|componentType|flexContainerConfig|children|styles|background|color|font|size|" +
        "styleTemplate|child|inputs|outputs||pipes|fxLayoutType|fxFlex|name|placeHolder|label|class|attributes|" +
        "fxLayoutAlign|flexElementConfig|fxLayoutWrap"
    );

    var buildInConstants = (
        "start|center|end|white|black|red|green|TextInputComponent|MoneyInputComponent|PercentageInputComponent|" +
        "AutoCompleteComponent|CardComponent|TableComponent|PieChartComponent|DOMContainerComponent|DOMElementComponent|" +
        "auto");

    var langClasses = ("");

    var HighlightRules = require("plugins/se.rwth.api.language/modes/language_highlight_rules");

    exports.MaCoCoVIZHighlightRules =
        HighlightRules("MaCoCoVIZ", keywords, buildInConstants, langClasses);
});