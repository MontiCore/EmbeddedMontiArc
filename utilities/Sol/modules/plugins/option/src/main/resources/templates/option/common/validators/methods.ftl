<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="delegator" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.OptionMethodDelegator" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.ValidatorPrinter" -->
<#-- @ftlvariable name="symbol" type="de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
${tc.signature("symbol")}
<#assign delegator = glex.getGlobalVar("option.delegator")>
<#assign printer = glex.getGlobalVar("option.printer")>
<#assign name = symbol.getName()>
<#assign options = delegator.getOptionSymbols(symbol)>
    public async validate(options: ${name?cap_first}Options, context: OptionsContext): Promise<${name?cap_first}Errors> {
        options = options || {};

        return {
            <#list options as option>
            <#assign optionName = option.getName()>
            ${optionName}: await this.validate${optionName?cap_first}(options.${optionName}, options)<#if option?has_next>,</#if>
            </#list>
        };
    }
<#list options as option>
${printer.printMethods(symbol, option)}
</#list>