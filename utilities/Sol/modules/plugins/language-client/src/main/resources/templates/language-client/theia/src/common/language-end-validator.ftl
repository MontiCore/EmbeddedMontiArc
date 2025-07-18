<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="declaration" type="de.monticore.lang.monticar.sol.grammars.language._symboltable.TemplateDeclarationSymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.ValidatorPrinter" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="end" type="java.lang.String" -->
<#-- @ftlvariable name="option" type="de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "declaration", "end")}// tslint:disable
<#assign printer = glex.getGlobalVar("option.printer")>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign name = declaration.getName()>
<#assign namecf = name?cap_first>
<#assign fullname = declaration.getFullName()>
import { injectable } from "inversify";
import { CommonTemplateValidator } from "@embeddedmontiarc/sol-runtime-templates/lib/common";
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";

import { ${namecf}Options, ${namecf}Errors } from "../../common/validators/${name?lower_case}-protocol";

@injectable()
export class ${namecf}${end}Validator<#if hasHandwrittenPeer>TOP</#if> extends CommonTemplateValidator<${namecf}Options, ${namecf}Errors> {
    public constructor() {
        super("${fullname}");
    }
    ${printer.printMethods(declaration)}
}
