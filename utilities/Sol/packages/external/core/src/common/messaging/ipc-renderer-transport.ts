/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Transport } from "modular-json-rpc";
import { ipcRenderer, IpcRendererEvent } from "electron";

export class IPCRendererTransport implements Transport {
    protected readonly path: string;

    public constructor(path: string) {
        this.path = path;
    }

    public SendUpstream(data: string): void {
        console.debug(`RENDERER TO MAIN: ${data}`);
        ipcRenderer.send(this.path, data);
    }

    public SetDownstreamCb(callback: (data: string) => void): void {
        ipcRenderer.on(this.path, (event: IpcRendererEvent, data: string) => {
            console.debug(`FROM MAIN TO RENDERER: ${data}`);
            callback(data);
        });
    }
}
