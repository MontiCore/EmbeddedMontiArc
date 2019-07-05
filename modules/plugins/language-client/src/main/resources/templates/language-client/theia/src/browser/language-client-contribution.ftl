<#-- @ftlvariable name="contribution" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template.TemplateContribution" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration" -->
${tc.signature("contribution")}
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign grammarName = configuration.getGrammarName()>
<#assign hasHandCodedPeer = contribution.hasHandCodedPeer()>
import { BaseLanguageClientContribution } from "@theia/languages/lib/browser";
import { injectable } from "inversify";
import { ${grammarName}Language } from "../common";

@injectable()
export class ${grammarName}ClientContribution<#if hasHandCodedPeer>Top</#if> extends BaseLanguageClientContribution {
    public readonly id: string = ${grammarName}Language.ID;
    public readonly name: string = ${grammarName}Language.NAME;

    protected get globPatterns(): string[] {
        return ["**/*.car"];
    }
}
