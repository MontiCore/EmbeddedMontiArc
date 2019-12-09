<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="ide" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.LocalAwareIDESymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "ide")}// tslint:disable
<#assign name = ide.getName()>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign ports = ide.getPorts()>
import { injectable } from "inversify";
import { DockerPortContribution, DockerPortRegistry } from "@embeddedmontiarc/sol-external-docker/lib/main";

@injectable()
export class ${name}DockerPortContribution<#if hasHandwrittenPeer>TOP</#if> implements DockerPortContribution {
    public registerPorts(registry: DockerPortRegistry): void {
        <#list ports as port>
        registry.registerPort(${port?c});
        </#list>
    }
}
