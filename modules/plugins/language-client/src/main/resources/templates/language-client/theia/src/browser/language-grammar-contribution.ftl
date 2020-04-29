<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration" -->
${tc.signature("template")}
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign rootSymbol = glex.getGlobalVar("rootSymbol")>
<#assign grammarName = configuration.getGrammarName()>
<#assign grammarNameLC = grammarName?lower_case>
<#assign extension = rootSymbol.getExtension().orElse(".")>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
/*
 * (c) https://github.com/MontiCore/monticore
 */
import { LanguageGrammarDefinitionContribution, TextmateRegistry } from "@theia/monaco/lib/browser/textmate";
import { injectable } from "inversify";
import { ${grammarName}Language } from "../common/${grammarNameLC}-protocol";

@injectable()
export class ${grammarName}GrammarContribution<#if hasHandwrittenPeer>Top</#if> implements LanguageGrammarDefinitionContribution {
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
        const grammar = require("../../data-gen/${grammarNameLC}.tmLanguage.json");

        monaco.languages.register({
            id: ${grammarName}Language.ID,
            aliases: [${grammarName}Language.NAME],
            extensions: ["${extension}"]
        });

        monaco.languages.setLanguageConfiguration(${grammarName}Language.ID, this.config);

        registry.registerTextmateGrammarScope("source.${extension}", {
            async getGrammarDefinition() {
                return {
                    format: "json",
                    content: grammar
                };
            }
        });

        registry.mapLanguageIdToTextmateGrammar(${grammarName}Language.ID, "source.${extension}");
    }
}
