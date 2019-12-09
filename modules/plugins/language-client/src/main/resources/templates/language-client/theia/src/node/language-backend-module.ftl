<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration" -->
<#-- @ftlvariable name="language" type="de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "language")}
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign grammarName = configuration.getGrammarName()>
<#assign grammarNameLC = grammarName?lower_case>
<#assign declarations = language.getLocalDeclarationSymbols()>
import { LanguageServerContribution } from "@theia/languages/lib/node";
import { TemplatesContribution } from "@embeddedmontiarc/sol-runtime-templates/lib/node/templates-registry";
import { ContainerModule } from "inversify";
import { ${grammarName}ServerContribution } from "./${grammarNameLC}-server-contribution";
import { ${grammarName}TemplatesContribution } from "./${grammarNameLC}-templates-contribution";
import { ${grammarName}BackendValidatorContribution } from "./${grammarNameLC}-backend-validator-contribution";
import { ValidatorContribution } from "@embeddedmontiarc/sol-runtime-options/lib/common";

<#list declarations as declaration>
<#assign dname = declaration.getName()>
import { ${dname?cap_first}BackendValidator } from "./validators/${dname?lower_case}-backend-validator";
</#list>

export default new ContainerModule(bind => {
    bind(${grammarName}ServerContribution).toSelf().inSingletonScope();
    bind(LanguageServerContribution).toService(${grammarName}ServerContribution);

    bind(${grammarName}TemplatesContribution).toSelf().inSingletonScope();
    bind(TemplatesContribution).toService(${grammarName}TemplatesContribution);

    bind(${grammarName}BackendValidatorContribution).toSelf().inSingletonScope();
    bind(ValidatorContribution).toService(${grammarName}BackendValidatorContribution);

    <#list declarations as declaration>
    <#assign dname = declaration.getName()>
    bind(${dname?cap_first}BackendValidator).toSelf().inSingletonScope();
    </#list>
});
