<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration" -->
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign grammarName = configuration.getGrammarName()>
<#assign grammarNameLC = grammarName?lower_case>
/*
 * (c) https://github.com/MontiCore/monticore
 */
export namespace ${grammarName}Language {
    export const ID: string = "${grammarNameLC}";
    export const NAME: string = "${grammarName}";
}
