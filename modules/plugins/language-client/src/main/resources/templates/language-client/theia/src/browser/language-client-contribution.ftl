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
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
/*
 * (c) https://github.com/MontiCore/monticore
 */
import { BaseLanguageClientContribution } from "@theia/languages/lib/browser";
import { injectable } from "inversify";
import { ${grammarName}Language } from "../common/${grammarNameLC}-protocol";

@injectable()
export class ${grammarName}ClientContribution<#if hasHandwrittenPeer>Top</#if> extends BaseLanguageClientContribution {
    public readonly id: string = ${grammarName}Language.ID;
    public readonly name: string = ${grammarName}Language.NAME;

    protected get globPatterns(): string[] {
        return ["**/*.${rootSymbol.getExtension().orElse(".")}"];
    }
}
