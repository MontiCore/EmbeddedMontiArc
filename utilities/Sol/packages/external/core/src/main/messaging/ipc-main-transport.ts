/*
 * (c) https://github.com/MontiCore/monticore
 */
import { bind } from "helpful-decorators";
import { Transport } from "modular-json-rpc";
import { ipcMain, IpcMainEvent } from "electron";
import { WebContents, webContents } from "electron";
import { WindowService } from "../window/window-service";

export const IPCMainTransportFactory = Symbol("IPCMainTransportFactory");
/**
 * A factory which should be used to create [[IPCMainTransport]] instances.
 */
export interface IPCMainTransportFactory {
    /**
     * Creates a [[IPCMainTransport]] instance from a given path.
     * @param path The path on which the [[IPCMainTransport]] should communicate.
     * @return The newly created [[IPCMainTransport]] instance.
     */
    (path: string): IPCMainTransport;
}

export class IPCMainTransport implements Transport {
    public static readonly OFFSET: number = 10000000;

    protected readonly path: string;
    protected readonly windows: WindowService;
    protected readonly queue: Map<number, number>;

    public constructor(path: string, windows: WindowService) {
        this.path = path;
        this.windows = windows;
        this.queue = new Map();
    }

    public SendUpstream(data: string): void {
        const message = JSON.parse(data);

        console.debug(`UPSTREAM: ${data}`);

        if (message.hasOwnProperty("result")) this.sendResponse(data);
        else if (message.hasOwnProperty("id")) this.sendRequest(data);
        else this.sendNotification(data);
    }

    public SetDownstreamCb(callback: (data: string) => void): void {
        ipcMain.on(this.path, (event: IpcMainEvent, data: string) => this.onMessage(event, data, callback));
    }

    protected sendResponse(data: string): void {
        const message = JSON.parse(data);
        const clientId = this.queue.get(message.id) || 0;
        const client = webContents.fromId(clientId);

        this.queue.delete(message.id);

        message.id = message.id - IPCMainTransport.OFFSET * clientId;

        if (client && !client.isDestroyed()) {
            console.debug(`RESPONSE MAIN TO RENDERER ${client.id}: ${JSON.stringify(message)}`);
            client.send(this.path, JSON.stringify(message));
        }
    }

    protected sendRequest(data: string): void {
        const window = this.windows.getMainWindow();
        const client = window ? window.webContents : undefined;

        if (client && !client.isDestroyed()) {
            console.debug(`REQUEST MAIN TO RENDERER ${client.id}: ${data}`);
            client.send(this.path, data);
        }
    }

    protected sendNotification(data: string): void {
        webContents.getAllWebContents().forEach(client => client.send(this.path, data));
    }

    protected handleRequest(message: { id: number; }, client: WebContents, callback: (data: string) => void): void {
        console.debug(`REQUEST RECEIVED`);
        this.queue.set(message.id, client.id);
        callback(JSON.stringify(message));
    }

    protected handleResponse(data: string, callback: (data: string) => void): void {
        console.debug(`RESPONSE RECEIVED`);
        callback(data);
    }

    protected handleNotification(data: string, callback: (data: string) => void): void {
        console.debug(`NOTIFICATION RECEIVED`);
        callback(data);
    }

    @bind
    protected onMessage(event: IpcMainEvent, data: string, callback: (data: string) => void): void {
        const client = event.sender;
        const message = JSON.parse(data);

        console.debug(`RENDERER ${client.id} to MAIN: ${data}`);

        if (message.hasOwnProperty("id")) {
            message.id = message.id + IPCMainTransport.OFFSET * client.id;

            if (message.hasOwnProperty("result")) return this.handleResponse(data, callback);
            else return this.handleRequest(message, client, callback);
        }

        return this.handleNotification(data, callback);
    }
}
