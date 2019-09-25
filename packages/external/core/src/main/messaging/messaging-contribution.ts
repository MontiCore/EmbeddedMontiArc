/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContributionProvider } from "@theia/core/lib/common/contribution-provider";
import { bind } from "helpful-decorators";
import { inject, injectable, named } from "inversify";
import { RPCNode, WSTransport } from "modular-json-rpc";
import { ApplicationContribution } from "../application";
import { Server, IncomingMessage } from "http";
import { Server as WebSocketServer } from "ws";
import { ConnectionHandler } from "../../common/messaging";
import { IPCMainTransportFactory } from "./ipc-main-transport";
import { Application } from "express";

import * as WebSocket from "ws";
import * as url from "url";

@injectable()
export class MessagingContribution implements ApplicationContribution {
    @inject(ContributionProvider) @named(ConnectionHandler)
    protected readonly provider: ContributionProvider<ConnectionHandler>;

    @inject(IPCMainTransportFactory) protected readonly factory: IPCMainTransportFactory;

    protected readonly handlers: Map<string, ConnectionHandler>;

    protected constructor() {
        this.handlers = new Map();
    }

    public async onConfigure(application: Application): Promise<void> {
        this.provider.getContributions().forEach(contribution => {
            const path = contribution.path;
            const node = new RPCNode(this.factory(path));

            node.requestTimeout = 60 * 60 * 1000;

            this.handlers.set(path, contribution);
            contribution.onConnection(node);
        });
    }

    public async onStart(server: Server): Promise<void> {
        const wss = new WebSocketServer({ server, perMessageDeflate: false });

        wss.on("connection", this.onConnection);
    }

    @bind
    protected onConnection(ws: WebSocket, request: IncomingMessage): void {
        const pathname = request.url && url.parse(request.url).pathname;
        const node = new RPCNode(new WSTransport(ws));

        if (pathname && this.handlers.has(pathname)) this.handlers.get(pathname)!.onConnection(node);
    }
}
