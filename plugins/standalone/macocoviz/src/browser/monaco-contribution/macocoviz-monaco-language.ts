/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/// <reference types="@theia/monaco/src/typings/monaco" />

import { DEFAULT_LANGUAGE_CONFIGURATION, DEFAULT_MONARCH_LANGUAGE } from "@elysium/languages/lib/browser";

export const configuration: monaco.languages.LanguageConfiguration = DEFAULT_LANGUAGE_CONFIGURATION;

export const monarchLanguage: monaco.languages.IMonarchLanguage = Object.assign({}, DEFAULT_MONARCH_LANGUAGE, {
    tokenPostfix: ".viz",
    keywords: [
        "vizualization", "componentType", "flexContainerConfig", "children", "styles", "background", "color", "font",
        "size", "styleTemplate", "child", "inputs", "outputs", "pipes", "fxLayoutType", "fxFlex", "name",
        "placeHolder", "label", "class", "attributes", "fxLayoutAlign", "flexElementConfig", "fxLayoutWrap"
    ],
    operators: [
        "start", "center", "end", "white", "black", "red", "green", "TextInputComponent", "MoneyInputComponent",
        "PercentageInputComponent", "AutoCompleteComponent", "CardComponent", "TableComponent", "PieChartComponent",
        "DOMContainerComponent", "DOMElementComponent", "auto"
    ]
});
