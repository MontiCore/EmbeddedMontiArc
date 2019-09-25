/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable, interfaces } from "inversify";
import { RPCNode } from "modular-json-rpc";
import { IPCRendererTransport } from "./ipc-renderer-transport";
import { RPCProxyFactory } from "./rpc-proxy-factory";

import Container = interfaces.Container;

@injectable()
export class IPCConnectionProvider {
    public static createProxy<Server extends object, Client extends object = {}>(container: Container, path: string, target?: Client): Server {
        return container.get(IPCConnectionProvider).createProxy(path, target);
    }

    public createProxy<Server extends object, Client extends object = {}>(path: string, target?: Client): Server {
        const node = new RPCNode(new IPCRendererTransport(path));
        const factory = new RPCProxyFactory<Client, Server>();

        node.requestTimeout = 60 * 60 * 1000;

        if (target) factory.setTarget(target);

        factory.listen(node);

        return factory.createProxy();
    }
}
