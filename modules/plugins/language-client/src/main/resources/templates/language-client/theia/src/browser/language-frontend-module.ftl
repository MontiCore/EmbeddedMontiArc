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
import { LanguageClientContribution } from "@theia/languages/lib/browser";
import { ContainerModule } from "inversify";
import { ${grammarName}ClientContribution } from "./${grammarNameLC}-client-contribution";
import { ${grammarName}GrammarContribution } from "./${grammarNameLC}-grammar-contribution";
import { LanguageGrammarDefinitionContribution } from "@theia/monaco/lib/browser/textmate";
import { ${grammarName}FrontendValidatorContribution } from "./${grammarNameLC}-frontend-validator-contribution";
import { ValidatorContribution } from "@embeddedmontiarc/sol-runtime-options/lib/common";

<#list declarations as declaration>
<#assign dname = declaration.getName()>
import { ${dname?cap_first}FrontendValidator } from "./validators/${dname?lower_case}-frontend-validator";
</#list>

export default new ContainerModule(bind => {
    bind(${grammarName}GrammarContribution).toSelf().inSingletonScope();
    bind(LanguageGrammarDefinitionContribution).to(${grammarName}GrammarContribution).inSingletonScope();

    bind(${grammarName}ClientContribution).toSelf().inSingletonScope();
    bind(LanguageClientContribution).toService(${grammarName}ClientContribution);

    bind(${grammarName}FrontendValidatorContribution).toSelf().inSingletonScope();
    bind(ValidatorContribution).toService(${grammarName}FrontendValidatorContribution);

    <#list declarations as declaration>
    <#assign dname = declaration.getName()>
    bind(${dname?cap_first}FrontendValidator).toSelf().inSingletonScope();
    </#list>
});
