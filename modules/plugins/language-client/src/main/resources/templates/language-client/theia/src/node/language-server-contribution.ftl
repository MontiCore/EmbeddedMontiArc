<#-- @ftlvariable name="contribution" type="de.monticore.lang.monticar.sol.plugins.common.plugin.generate.template.TemplateContribution" -->
<#-- @ftlvariable name="tc" type="de.monticore.generating.templateengine.TemplateController" -->
<#-- @ftlvariable name="glex" type="de.monticore.generating.templateengine.GlobalExtensionManagement" -->
<#-- @ftlvariable name="configuration" type="de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration" -->
${tc.signature("contribution")}
<#assign configuration = glex.getGlobalVar("configuration")>
<#assign grammarName = configuration.getGrammarName()>
<#assign hasHandCodedPeer = contribution.hasHandCodedPeer()>
import { injectable } from "inversify";
import { AddressInfo } from "net";
import { ${grammarName}Language } from "../common";
import { BaseLanguageServerContribution, IConnection } from "@theia/languages/lib/node";

import * as FileSystem from "fs-extra";

@injectable()
export class ${grammarName}ServerContribution<#if hasHandCodedPeer>Top</#if> extends BaseLanguageServerContribution {
    public readonly id: string = ${grammarName}Language.ID;
    public readonly name: string = ${grammarName}Language.NAME;

    public async start(clientConnection: IConnection): Promise<void> {
        // TODO: Change Environmental Variable to Path Resolution.
        const envVariable = process.env.${grammarName}_LANGUAGE_SERVER;

        if (envVariable) return this.doStart(clientConnection, envVariable);
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