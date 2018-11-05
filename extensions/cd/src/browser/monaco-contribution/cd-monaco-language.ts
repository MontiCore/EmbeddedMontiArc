/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/// <reference types="@theia/monaco/src/typings/monaco" />

import { DEFAULT_LANGUAGE_CONFIGURATION, DEFAULT_MONARCH_LANGUAGE } from "@elysium/languages/lib/browser";

export const configuration: monaco.languages.LanguageConfiguration = DEFAULT_LANGUAGE_CONFIGURATION;

export const monarchLanguage: monaco.languages.IMonarchLanguage = Object.assign({}, DEFAULT_MONARCH_LANGUAGE, {
    tokenPostfix: ".cd",
    keywords: [
        "void", "boolean", "byte", "short", "int", "long", "char", "float", "double", "extends", "super",
        "import", "package", "classdiagram", "class", "implements", "interface", "enum", "throws", "association",
        "composition", "abstract", "final", "static", "private", "protected", "public", "derived"
    ],
    operators: [
        "null", "true", "false"
    ]
});
