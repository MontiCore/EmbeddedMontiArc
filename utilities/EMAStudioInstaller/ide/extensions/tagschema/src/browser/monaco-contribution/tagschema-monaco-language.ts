/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/// <reference types="@theia/monaco/src/typings/monaco" />

import { DEFAULT_LANGUAGE_CONFIGURATION, DEFAULT_MONARCH_LANGUAGE } from "@elysium/languages/lib/browser";

export const configuration: monaco.languages.LanguageConfiguration = DEFAULT_LANGUAGE_CONFIGURATION;

export const monarchLanguage: monaco.languages.IMonarchLanguage = Object.assign({}, DEFAULT_MONARCH_LANGUAGE, {
    tokenPostfix: ".tagschema",
    keywords: [
        "package", "tagschema", "Component", "Port", "Connector", "ExpandedComponentInstance", "for", "private", "tagtype"
    ],
    operators: [
        "undefined", "Number", "String", "Boolean"
    ]
});
