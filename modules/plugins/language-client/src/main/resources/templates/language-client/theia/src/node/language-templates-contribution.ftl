<#-- @ftlvariable name="ast" type="de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit" -->
<#-- @ftlvariable name="extractor" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.ld.LDExtractor" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="templates" type="java.util.List<org.json.JSONObject>" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration" -->
${tc.signature("extractor")}
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign grammarName = configuration.getGrammarName()>
import {
    TemplatesContribution,
    TemplatesRegistry
} from "@embeddedmontiarc/sol-runtime-templates/src/node/templates-contribution";
import { injectable } from "inversify";
import * as path from "path";

@injectable()
export class ${grammarName}TemplatesContribution implements TemplatesContribution {
    public registerTemplates(registry: TemplatesRegistry): void {
        <#list extractor.getTemplates(ast) as template>
        registry.registerTemplate({
            id: "${extractor.getIdentifier(template)}",
            extension: ".${configuration.getFileExtension()}",
            path: path.resolve(__dirname, "..", "..", "templates", "${extractor.getPath(template)}"),
            label: "${extractor.getLabel(template)}",
            elements: ${extractor.getElements(template).toString(4)}
        });
        </#list>
    }
}
