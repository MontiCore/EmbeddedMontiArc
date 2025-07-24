/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/// <reference types="@theia/monaco/src/typings/monaco" />

import { DEFAULT_LANGUAGE_CONFIGURATION, DEFAULT_MONARCH_LANGUAGE } from "@elysium/languages/lib/browser";

export const configuration: monaco.languages.LanguageConfiguration = DEFAULT_LANGUAGE_CONFIGURATION;

export const monarchLanguage: monaco.languages.IMonarchLanguage = Object.assign({}, DEFAULT_MONARCH_LANGUAGE, {
    tokenPostfix: ".mc4",
    keywords: [
        "package", "component", "grammar", "extends", "import", "options", "start", "fragment", "comment", "token",
        "enum", "external", "interface", "astextends", "abstract", "implements", "astimplements", "init", "EOF",
        "MCA", "concept", "astrule", "ast", "symbol", "scope", "method"
    ],
    operators: [
        "follow", "min", "max", "public", "private", "protected", "final", "static", "throws"
    ]
});
