<#-- (c) https://github.com/MontiCore/monticore -->
<#-- @ftlvariable name="rootPackage" type="de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage" -->
<#-- @ftlvariable name="template" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration" -->
${tc.signature("template")}
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign rootPackage = glex.getGlobalVar("rootPackage")>
<#assign grammarName = configuration.getGrammarName()>
<#assign grammarNameLC = grammarName?lower_case>
<#assign hasHandwrittenPeer = template.hasHandwrittenPeer()>
<#assign serverPath = rootPackage.getDirectory("server").get()!"server">
/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { AddressInfo } from "net";
import { ${grammarName}Language } from "../common/${grammarNameLC}-protocol";
import { BaseLanguageServerContribution, IConnection } from "@theia/languages/lib/node";

import * as FileSystem from "fs-extra";
import * as Path from "path";

@injectable()
export class ${grammarName}ServerContribution<#if hasHandwrittenPeer>Top</#if> extends BaseLanguageServerContribution {
    public readonly id: string = ${grammarName}Language.ID;
    public readonly name: string = ${grammarName}Language.NAME;

    public async start(clientConnection: IConnection): Promise<void> {
        const pathToJar = Path.resolve(__dirname, "..", "..", "${serverPath}", "${grammarName}.jar");

        if (await FileSystem.pathExists(pathToJar)) return this.doStart(clientConnection, pathToJar);
    }

    protected async doStart(clientConnection: IConnection, pathToJar: string): Promise<void> {
        const server = await this.startSocketServer();
        const socket = this.accept(server);
        const address = server.address() as AddressInfo;
        const command = "java";
        const args = ["-jar", pathToJar, "-port", `${r"${address.port}"}`];
        const serverConnection = await this.createProcessSocketConnection(socket, socket, command, args);

        this.forward(clientConnection, serverConnection);
    }
}
