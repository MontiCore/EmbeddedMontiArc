<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="module" type="de.monticore.lang.monticar.sol.grammars.ide._symboltable.ModuleTypeSymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer.ImportPrinter" -->
<#-- @ftlvariable name="solPackage" type="de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage" -->
<#-- @ftlvariable name="delegator" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.IDEMethodDelegator" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "module")}// tslint:disable
<#assign printer = glex.getGlobalVar("ide.printer")>
<#assign delegator = glex.getGlobalVar("ide.delegator")>
<#assign solPackage = glex.getGlobalVar("solPackage")>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign name = module.getName()>
<#assign fullname = module.getFullName()>
<#assign subModules = module.getModuleSymbols()>
<#assign subModuleTypes = module.getModuleTypeSymbols()>
<#assign configurations = module.getConfigurationSymbols()>
<#assign artifactWrites = module.getArtifactWriteSymbols()>
<#assign moduleWrites = module.getModuleWriteSymbols()>
import { injectable<#if artifactWrites?has_content || moduleWrites?has_content>, inject</#if> } from "inversify";
import { CommonModuleCreator } from "@embeddedmontiarc/sol-runtime-modules/lib/node";
import { Configuration } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";
import { ModuleContext } from "@embeddedmontiarc/sol-runtime-modules/lib/common";

<#if configurations?has_content>import { v4 } from "uuid";</#if>

import * as path from "path";
<#if artifactWrites?has_content>import * as fs from "fs-extra";</#if>

<#list subModuleTypes as subModuleType>
<#assign subName = subModuleType.getName()>
<#assign qualifiedClass = printer.printQualifiedClassPrefix(subModuleType)>
import {
    ${subName}ModuleCreator as ${qualifiedClass}ModuleCreator
} from "${printer.printImport(solPackage, subModuleType, "ide", "../../../node/modules/%s/creator")}";
</#list>

<#list artifactWrites as artifactWrite>
<#assign artifact = artifactWrite.getReferencedArtifact().get()>
<#assign artifactName = artifact.getName()>
<#assign qualifiedClass = printer.printQualifiedClassPrefix(artifact)>
import {
    ${artifactName}Artifact
} from "${printer.printImport(solPackage, artifact, "ac", "../../../node/%s/artifact")}";
</#list>

@injectable()
export class ${name}ModuleCreator<#if hasHandwrittenPeer>TOP</#if> extends CommonModuleCreator {
    <#list subModules as subModule>
    <#assign subName = subModule.getName()>
    <#assign subModuleType = subModule.getTypeSymbol()>
    <#assign qualifiedClass = printer.printQualifiedClassPrefix(subModuleType.get())>
    @inject(${qualifiedClass}ModuleCreator) protected readonly ${subName}Creator: ${qualifiedClass}ModuleCreator;
    </#list>

    <#list artifactWrites as artifactWrite>
    <#assign artifact = artifactWrite.getReferencedArtifact().get()>
    <#assign artifactName = artifact.getName()>
    @inject(${artifactName}Artifact) protected readonly ${artifactName?uncap_first}Artifact: ${artifactName}Artifact;
    </#list>

    public constructor() {
        super("${fullname}");
    }

    public async createModules(destination: string, context: ModuleContext): Promise<Configuration[]> {
        path.resolve("."); // Hack to circumvent TypeScript compiler error in case path is not used.

        const configurations = [
            <#list configurations as configuration>
            <#assign cname = configuration.getDisplayName().get()>
            <#assign typeId = configuration.getTypeSymbol().get().getFullName()>
            {
                uuid: v4(),
                typeId: "${typeId}",
                name: "${configuration.getDisplayName().get()}",
                options: {
                    <#list delegator.getOptionFillSymbols(configuration) as fill>
                    ${fill.getName()}: ${printer.printOptionFillValue(fill)}<#if fill?has_next>,</#if>
                    </#list>
                }
            }<#if configuration?has_next>,</#if>
            </#list>
        ];

        const subConfigurations = await Promise.all([
            <#list moduleWrites as moduleWrite>
            <#assign path = moduleWrite.getRelativePath()>
            <#assign moduleName = moduleWrite.getName()>
            this.${moduleName}Creator.createModules(path.resolve(destination, "${path}"))<#if moduleWrite?has_next>,</#if>
            </#list>
        ]);

        await Promise.all([
            <#list artifactWrites as artifactWrite>
            <#assign path = artifactWrite.getRelativePath()>
            <#assign artifact = artifactWrite.getReferencedArtifact().get()>
            <#assign artifactName = artifact.getName()>
            fs.copy(this.${artifactName?uncap_first}Artifact.path, path.resolve(destination, "${path}"))<#if artifactWrite?has_next>,</#if>
            </#list>
        ]);

        return [...configurations, ...subConfigurations];
    }
}
