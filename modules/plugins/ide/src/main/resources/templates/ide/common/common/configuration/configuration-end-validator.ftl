<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer.ImportPrinter" -->
<#-- @ftlvariable name="oprinter" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.ValidatorPrinter" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="end" type="java.lang.String" -->
<#-- @ftlvariable name="option" type="de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "configuration", "end")}// tslint:disable
<#assign printer = glex.getGlobalVar("ide.printer")>
<#assign oprinter = glex.getGlobalVar("option.printer")>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign name = configuration.getName()>
<#assign fullname = configuration.getFullName()>
import { injectable } from "inversify";
import { CommonConfigurationValidator } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";

<#assign qualifiedPath = printer.printQualifiedPath(configuration)>
import { ${name}Options, ${name}Errors } from "../../../common/configurations/${qualifiedPath}/protocol";

@injectable()
export class ${name}${end}Validator<#if hasHandwrittenPeer>TOP</#if> extends CommonConfigurationValidator<${name}Options, ${name}Errors> {
    public constructor() {
        super("${fullname}");
    }
    ${oprinter.printMethods(configuration)}
}
