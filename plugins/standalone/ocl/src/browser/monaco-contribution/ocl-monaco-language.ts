/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/// <reference types="@theia/monaco/src/typings/monaco" />

import { DEFAULT_LANGUAGE_CONFIGURATION, DEFAULT_MONARCH_LANGUAGE } from "@elysium/languages/lib/browser";

export const configuration: monaco.languages.LanguageConfiguration = DEFAULT_LANGUAGE_CONFIGURATION;

export const monarchLanguage: monaco.languages.IMonarchLanguage = Object.assign({}, DEFAULT_MONARCH_LANGUAGE, {
    tokenPostfix: ".ocl",
    keywords: [
        "void", "boolean", "byte", "short", "int", "long", "char", "float", "double", "extends", "super",
        "import", "public", "private", "protected", "final", "abstract", "local", "derived", "readonly",
        "static", "context", "inv", "in", "pre", "post", "new", "throws", "if", "then", "else", "typeif",
        "instanceof", "forall", "exists", "any", "let", "iterate", "isin", "implies", "this", "result",
        "isnew", "defined", "package"
    ],
    operators: [
        "Set", "List", "Collection", "null", "true", "false"
    ]
});
