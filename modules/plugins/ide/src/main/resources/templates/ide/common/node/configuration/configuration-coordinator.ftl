<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="printer" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer.ImportPrinter" -->
<#-- @ftlvariable name="delegator" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.IDEMethodDelegator" -->
<#-- @ftlvariable name="sorter" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.order.OrderService" -->
<#-- @ftlvariable name="solPackage" type="de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "configuration")}// tslint:disable
<#assign printer = glex.getGlobalVar("ide.printer")>
<#assign delegator = glex.getGlobalVar("ide.delegator")>
<#assign sorter = glex.getGlobalVar("ide.sorter")>
<#assign solPackage = glex.getGlobalVar("solPackage")>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign name = configuration.getName()>
<#assign fullName = configuration.getFullName()>
<#assign options = name + "Options">
<#assign tasks = configuration.getTaskSymbols()>
<#assign subConfigurations = delegator.getConfigurationSymbols(configuration)>
<#assign subConfigurationTypes = delegator.getUniqueConfigurationTypeSymbols(configuration)>
<#assign orderedTasks = sorter.orderTasks(configuration)>
<#assign zeroConfigurations = delegator.getZeroConfigurationSymbols(configuration)>
<#assign positiveConfigurations = sorter.orderConfigurations(delegator.getPositiveConfigurationSymbols(configuration))>
<#assign negativeConfigurations = sorter.orderConfigurations(delegator.getNegativeConfigurationSymbols(configuration))>
import { injectable<#if subConfigurations?has_content>, inject</#if> } from "inversify";
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationToken } from "@theia/core/lib/common";
import { CommonConfigurationCoordinator } from "@embeddedmontiarc/sol-runtime-configurations/lib/node";

<#assign qualifiedPath = printer.printQualifiedPath(configuration)>
import {
    ${options}
} from "../../../common/configurations/${qualifiedPath}/protocol";

<#list subConfigurationTypes as subConfigurationType>
<#assign subName = subConfigurationType.getName()>
<#assign qualifiedClass = printer.printQualifiedClassPrefix(subConfigurationType)>
<#assign qualifiedPath = printer.printQualifiedPath(subConfigurationType)>
import {
    ${subName}Options as ${qualifiedClass}Options
} from "../../../common/configurations/${qualifiedPath}/protocol";
import {
    ${subName}Coordinator as ${qualifiedClass}Coordinator
} from "${printer.printImport(solPackage, subConfigurationType, "ide", "../../../node/configurations/%s/coordinator")}";
</#list>

@injectable()
export class ${name}Coordinator<#if hasHandwrittenPeer>TOP</#if> extends CommonConfigurationCoordinator<${options}> {
    <#list subConfigurations as subConfiguration>
    <#assign subConfigurationName = subConfiguration.getName()>
    <#assign subConfigurationType = subConfiguration.getTypeSymbol()>
    <#if subConfigurationType.isPresent()>
    <#assign qualifiedClass = printer.printQualifiedClassPrefix(subConfigurationType.get())>
    @inject(${qualifiedClass}Coordinator)
    protected readonly ${subConfigurationName}: ${qualifiedClass}Coordinator;
    </#if>
    </#list>

    public constructor() {
        super("${fullName}");
    }

    public async run(uuid: string, options: ${options}, context: OptionsContext, token: CancellationToken): Promise<void> {
        <#if negativeConfigurations?has_content>
        <#list negativeConfigurations as negativeConfiguration>
        await Promise.all([
            <#list negativeConfiguration as nc>
            <#assign ncname = nc.getName()>
            this.${ncname}.run(uuid, this.${ncname}Options(options), context, token)<#if nc?has_next>,</#if>
            </#list>
        ]);
        </#list>
        </#if>

        <#if orderedTasks?has_content || zeroConfigurations?has_content>
        await Promise.all([<#if orderedTasks?has_content>(async () => {
                <#list orderedTasks as orderedTask>
                await Promise.all([
                    <#list orderedTask as task>
                    <#assign methodSuffix = printer.printMethodNameSuffix(task)>
                    this.run${methodSuffix}(uuid, options, context, token)<#if task?has_next>,</#if>
                    </#list>
                ]);
                </#list>
            })()</#if><#if zeroConfigurations?has_content>,
            Promise.all([
                <#list zeroConfigurations as zeroConfiguration>
                <#assign zeroConfigurationName = zeroConfiguration.getName()>
                this.${zeroConfigurationName}.run(uuid, this.${zeroConfigurationName}Options(options), context, token)<#if zeroConfiguration?has_next>,</#if>
                </#list>
            ])
            </#if>
        ]);
        </#if>

        <#if positiveConfigurations?has_content>
        <#list positiveConfigurations as positiveConfiguration>
        await Promise.all([
            <#list positiveConfiguration as pc>
            <#assign pcname = pc.getName()>
            this.${pcname}.run(uuid, this.${pcname}Options(options), context, token)<#if pc?has_next>,</#if>
            </#list>
        ]);
        </#list>
        </#if>
    }

    <#list tasks as task>
    <#assign taskName = task.getName()>
    <#assign methodSuffix = printer.printMethodNameSuffix(task)>
    protected run${methodSuffix}(uuid: string, options: ${options}, context: OptionsContext, token: CancellationToken): Promise<void> {
        <#if task.isFrontend()>
        return this.frontend.run(uuid, this.id, "${taskName}", options, context);
        <#else>
        return this.backend.run(uuid, this.id, "${taskName}", options, context, token);
        </#if>
    }
    </#list>

    <#list subConfigurations as subConfiguration>
    <#assign subConfigurationType = subConfiguration.getTypeSymbol()>
    <#if subConfigurationType.isPresent()>
    <#assign inherits = delegator.getOptionInheritSymbols(subConfiguration)>
    <#assign fills = delegator.getOptionFillSymbols(subConfiguration)>
    <#assign subConfigurationType = subConfigurationType.get()>
    <#assign qualifiedClass = printer.printQualifiedClassPrefix(subConfigurationType)>
    <#assign subConfigurationName = subConfiguration.getName()>
    protected ${subConfigurationName}Options(options: ${options}): ${qualifiedClass}Options {
        return {
            <#list fills as fill>
            ${fill.getName()}: ${printer.printOptionFillValue(fill)}
            </#list><#if fills?has_content && inherits?has_content>,</#if>
            <#list inherits as inherit>
            <#assign option = inherit.getOptionSymbol()>
            <#assign parent = inherit.getParentSymbol()>
            <#if option.isPresent() && parent.isPresent()>
            <#assign option = option.get()>
            <#assign parent = parent.get()>
            ${option.getName()}: options.${parent.getName()}<#if inherit?has_next>,</#if>
            </#if>
            </#list>
        };
    }

    </#if>
    </#list>
}
