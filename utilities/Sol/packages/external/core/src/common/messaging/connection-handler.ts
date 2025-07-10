/*
 * (c) https://github.com/MontiCore/monticore
 */
import { RPCNode } from "modular-json-rpc";
import { RPCProxyFactory } from "./rpc-proxy-factory";

export const ConnectionHandler = Symbol("ConnectionHandler");
/**
 * An interface to be implemented by a class which handles connections for a given path.
 */
export interface ConnectionHandler {
    /**
     * The path for which the handler is responsible.
     */
    readonly path: string;

    /**
     * A method which is triggered when a node is connecting to the server.
     * @param node The node which is currently connecting to the server.
     */
    onConnection(node: RPCNode): void;
}

/*
 * This class has been inspired by Theia's JsonRpcConnectionHandler.
 */
export class RPCConnectionHandler<Server extends object, Client extends object = {}> implements ConnectionHandler {
    public readonly path: string;

    protected readonly targetFactory: (proxy: Client) => Server;

    public constructor(path: string, targetFactory: (proxy: Client) => Server) {
        this.path = path;
        this.targetFactory = targetFactory;
    }

    public onConnection(node: RPCNode): void {
        const factory = new RPCProxyFactory<Server, Client>();
        const proxy = factory.createProxy();

        factory.setTarget(this.targetFactory(proxy));
        factory.listen(node);
    }
}
