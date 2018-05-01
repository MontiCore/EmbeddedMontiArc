/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, interfaces } from "inversify";
import { JsonRpcProxy, JsonRpcProxyFactory, ConnectionHandler } from "@theia/core/lib/common";
import { ConsoleLogger, createWebSocketConnection, IWebSocket } from "vscode-ws-jsonrpc";

@injectable()
export class WorkerConnectionProvider {
    protected static worker: Worker;

    public static createProxy<T extends object>(container: interfaces.Container, path: string, target?: object): JsonRpcProxy<T> {
        return container.get(WorkerConnectionProvider).createProxy<T>(path, target);
    }

    public createProxy<T extends object>(path: string, target?: object): JsonRpcProxy<T> {
        const factory = new JsonRpcProxyFactory<T>(target);

        this.listen({
            path,
            onConnection: callback => factory.listen(callback)
        });

        return factory.createProxy();
    }

    public listen(handler: ConnectionHandler): void {
        const worker = this.createWorker();
        const logger = new ConsoleLogger();

        worker.onerror = error => logger.error('' + error);

        const socket = this.toSocket(worker);
        const connection = createWebSocketConnection(socket, logger);

        handler.onConnection(connection);
    }

    public createWorker(): Worker {
        return WorkerConnectionProvider.worker
            ? WorkerConnectionProvider.worker
            : WorkerConnectionProvider.worker = new Worker("./bundle.worker.js");
    }

    protected toSocket(worker: Worker): IWebSocket {
        return {
            send: content => worker.postMessage(content),
            onMessage: callback => worker.onmessage = event => callback(event.data),
            onError: callback => worker.onerror = event => {
                if ("message" in event) callback(event.message);
            },
            onClose: () => {},
            dispose: () => {}
        };
    }
}
