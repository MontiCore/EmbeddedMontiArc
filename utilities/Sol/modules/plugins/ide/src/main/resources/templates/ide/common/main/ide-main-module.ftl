<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="ide" type="de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable.LocalAwareIDESymbol" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "ide")}// tslint:disable
<#assign name = ide.getName()>
import { ContainerModule } from "inversify";
import { DockerPortContribution } from "@embeddedmontiarc/sol-external-docker/lib/main";

import { ${name}DockerPortContribution } from "./${name?lower_case}-docker-port-contribution";

export default new ContainerModule(bind => {
    bind(DockerPortContribution).to(${name}DockerPortContribution).inSingletonScope();
});
