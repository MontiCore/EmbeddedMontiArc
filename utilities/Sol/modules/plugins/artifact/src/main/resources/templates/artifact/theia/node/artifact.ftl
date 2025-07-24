<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="artifact" type="de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactSymbol" -->
<#-- @ftlvariable name="package" type="de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage" -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
/*
 * (c) https://github.com/MontiCore/monticore
 */
${tc.signature("template", "package", "artifact")}//tslint:disable
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign name = artifact.getName()>
<#assign qualifiedFolder = artifact.getFullName()?lower_case>
<#assign path = artifact.getPath().get()>
<#assign directory = package.getDirectory("artifacts").get()>
import { injectable } from "inversify";
import { CommonArtifact } from "@embeddedmontiarc/sol-runtime-artifact/lib/node";

import * as path from "path";

@injectable()
export class ${name}Artifact<#if hasHandwrittenPeer>TOP</#if> extends CommonArtifact {
    public constructor() {
        const artifactsDirectory = path.resolve(__dirname, "../../..", "${directory}");
        const basename = path.basename("${path}");
        const absolutePath = path.resolve(artifactsDirectory, "${qualifiedFolder}", basename);

        super(absolutePath);
    }
}