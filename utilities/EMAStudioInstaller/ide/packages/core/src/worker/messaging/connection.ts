/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import {ConsoleLogger, createWebSocketConnection, IWebSocket, MessageConnection} from "vscode-ws-jsonrpc";

export interface IWorkerOptions {
    readonly path?: string;
}

export function createWorkerConnection(options: IWorkerOptions, onConnect: (connection: MessageConnection) => void): void {
    const logger = new ConsoleLogger();
    // tslint:disable-next-line:no-any
    const socket = toIWebSocket(self as any);
    const connection = createWebSocketConnection(socket, logger);

    onConnect(connection);
}

export function toIWebSocket(worker: Worker): IWebSocket {
    return <IWebSocket>{
        send: content => worker.postMessage(content),
        onMessage: callback => worker.onmessage = event => callback(event.data),
        onError: callback => worker.onerror = callback,
        onClose: () => {},
        dispose: () => {}
    };
}
