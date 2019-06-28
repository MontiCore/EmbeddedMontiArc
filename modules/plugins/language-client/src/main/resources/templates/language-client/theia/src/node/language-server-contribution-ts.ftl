/*
 * Copyright (C) ${year} SE RWTH.
 */

import { injectable } from "inversify";
import { AddressInfo } from "net";
import { ${Grammar}Language } from "../common";
import { BaseLanguageServerContribution, IConnection } from "@theia/languages/lib/node";

import * as FileSystem from "fs-extra";

@injectable()
export class ${Grammar}ServerContribution extends BaseLanguageServerContribution {
    public readonly id: string = ${Grammar}Language.ID;
    public readonly name: string = ${Grammar}Language.NAME;

    public async start(clientConnection: IConnection): Promise<void> {
        // TODO: Change Environmental Variable to Path Resolution.
        const envVariable = process.env.${Grammar}_LANGUAGE_SERVER;

        if (envVariable) return this.doStart(clientConnection, envVariable);
    }

    protected async doStart(clientConnection: IConnection, pathToJar: string): Promise<void> {
        const server = await this.startSocketServer();
        const socket = this.accept(server);
        const address = server.address() as AddressInfo;
        const command = "java";
        const args = ["-jar", pathToJar, "-port", `${address.port}`];
        const serverConnection = await this.createProcessSocketConnection(socket, socket, command, args);

        this.forward(clientConnection, serverConnection);
    }
}