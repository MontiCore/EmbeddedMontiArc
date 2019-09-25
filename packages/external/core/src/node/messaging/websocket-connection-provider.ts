/*
 * (c) https://github.com/MontiCore/monticore
 */
import { inject, injectable } from "inversify";
import { RPCNode, WSTransport } from "modular-json-rpc";
import { RPCProxyFactory } from "../../common/messaging";
import { MessagingCliContribution } from "./messaging-cli-contribution";

import * as WebSocket from "ws";

/**
 * @ignore
 */
@injectable()
export class WebSocketConnectionProvider {
    @inject(MessagingCliContribution) protected readonly cli: MessagingCliContribution;

    public createProxy<Server extends object, Client extends object = {}>(path: string, target?: Client): Server {
        const host = this.cli.getExternalHost();
        const websocket = new WebSocket(`ws://${host}${path}`);
        const node = new RPCNode(new WSTransport(websocket));
        const factory = new RPCProxyFactory<Client, Server>();

        if (target) factory.setTarget(target);

        factory.listen(node);

        return factory.createProxy();
    }
}
