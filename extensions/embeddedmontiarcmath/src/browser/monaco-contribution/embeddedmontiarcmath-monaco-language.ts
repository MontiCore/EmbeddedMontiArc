/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

/// <reference types="@theia/monaco/src/typings/monaco" />

import { DEFAULT_LANGUAGE_CONFIGURATION, DEFAULT_MONARCH_LANGUAGE } from "@elysium/languages/lib/browser";

export const configuration: monaco.languages.LanguageConfiguration = DEFAULT_LANGUAGE_CONFIGURATION;

export const monarchLanguage: monaco.languages.IMonarchLanguage = Object.assign({}, DEFAULT_MONARCH_LANGUAGE, {
    tokenPostfix: ".emam",
    keywords: [
        "package", "component", "implements", "port", "ports", "in", "out", "instance", "connect", "autoconnect",
        "type", "off", "autoinstantiate", "on", "import", "implementation", "for", "end", "elseif", "else", "if"
    ],
    operators: []
});
