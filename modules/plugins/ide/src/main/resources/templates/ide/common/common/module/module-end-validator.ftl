<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="module" type="de.monticore.lang.monticar.sol.grammars.ide._symboltable.ModuleTypeSymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="end" type="java.lang.String" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "module", "end")}// tslint:disable
<#assign name = module.getName()>
<#assign fullname = module.getFullName()>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
import { injectable } from "inversify";
import { CommonModuleValidator } from "@embeddedmontiarc/sol-runtime-modules/lib/common";

@injectable()
export class ${name}${end}Validator<#if hasHandwrittenPeer>TOP</#if> extends CommonModuleValidator {
    public constructor() {
        super("${fullname}");
    }

    public validate(destination: string): Promise<string | true> {
        return Promise.resolve(true);
    }
}
