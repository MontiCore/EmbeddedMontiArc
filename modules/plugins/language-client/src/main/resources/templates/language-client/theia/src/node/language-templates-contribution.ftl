<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="rootPackage" type="de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage" -->
<#-- @ftlvariable name="ast" type="de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit" -->
<#-- @ftlvariable name="extractor" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.generator.lc.LCExtractor" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="templates" type="java.util.List<org.json.JSONObject>" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration" -->
<#-- @ftlvariable name="serializer" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.serializer.OptionsSerializer" -->
${tc.signature("extractor")}
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign rootPackage = glex.getGlobalVar("rootPackage")>
<#assign serializer = glex.getGlobalVar("option.serializer")>
<#assign grammarName = configuration.getGrammarName()>
<#assign templatesDirectory = rootPackage.getDirectory("templates").get()!"templates">
/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";

import {
    TemplatesContribution,
    TemplatesRegistry
} from "@embeddedmontiarc/sol-runtime-templates/lib/node/templates-registry";

import * as path from "path";

@injectable()
export class ${grammarName}TemplatesContribution implements TemplatesContribution {
    public registerTemplates(registry: TemplatesRegistry): void {
        <#list extractor.getTemplateDeclarations(ast) as declaration>
        registry.registerTemplate({
            id: "${extractor.getIdentifier(declaration)}",
            extension: ".${configuration.getFileExtension()}",
            path: path.resolve(__dirname, "..", "..", "${templatesDirectory}", "${extractor.getPath(declaration)}"),
            label: "${extractor.getLabel(declaration)}",
            options: ${serializer.serialize(declaration)}
        });
        </#list>

        <#list extractor.getTemplateUndeclarations(ast) as undeclaration>
        registry.unregisterTemplate("${extractor.getIdentifier(undeclaration)}");
        </#list>
    }
}
