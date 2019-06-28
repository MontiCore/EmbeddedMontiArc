/*
 * Copyright (C) ${year} SE RWTH.
 */

import { LanguageGrammarDefinitionContribution, TextmateRegistry } from "@theia/monaco/lib/browser/textmate";
import { injectable } from "inversify";
import { ${Grammar}Language } from "../common";

@injectable()
export class ${Grammar}GrammarContribution implements LanguageGrammarDefinitionContribution {
    protected readonly config: monaco.languages.LanguageConfiguration = {
        comments: {
            lineComment: "//",
            blockComment: ["/*", "*/"]
        },
        brackets: [
            ['{', '}'],
            ['[', ']'],
            ['(', ')']
        ],
        autoClosingPairs: [
            { open: '[', close: ']' },
            { open: '{', close: '}' },
            { open: '(', close: ')' },
            { open: '\'', close: '\'', notIn: ['string', 'comment'] },
            { open: '"', close: '"', notIn: ['string'] },
            { open: '/*', close: ' */', notIn: ['string'] }
        ],
        surroundingPairs: [
            { open: '{', close: '}' },
            { open: '[', close: ']' },
            { open: '(', close: ')' },
            { open: '"', close: '"' },
            { open: '\'', close: '\'' },
        ]
    };

    public registerTextmateLanguage(registry: TextmateRegistry): void {
        const grammar = require("../../data/${grammar}.tmLanguage.json");

        monaco.languages.register({
            id: ${Grammar}Language.ID,
            aliases: [${Grammar}Language.NAME],
            extensions: [".${extension}"]
        });

        monaco.languages.setLanguageConfiguration(${Grammar}Language.ID, this.config);

        registry.registerTextmateGrammarScope("source.${extension}", {
            async getGrammarDefinition() {
                return {
                    format: "json",
                    content: grammar
                };
            }
        });

        registry.mapLanguageIdToTextmateGrammar(${Grammar}Language.ID, "source.${extension}");
    }
}
