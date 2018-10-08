/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";

// tslint:disable:no-any

export interface RelayMessage {
    path: string;
    type: any;
    message: any;
}

@injectable()
export class RelayService {
    protected readonly ipcRenderer: any;

    public constructor() {
        const windowAny = window as any;
        const isElectron = !!windowAny.process;

        if (isElectron) {
            const electron = windowAny.require("electron");

            this.ipcRenderer = electron.ipcRenderer;
        } else {
            console.warn("Electron Environment Detection failed: Inter-Project Communication disabled.");
        }
    }

    public onMessage(handler: Function): void {
        if (this.ipcRenderer) this.ipcRenderer.on(RelayService.CHANNEL, (event: any, message: any) => handler(message));
        else window.top.addEventListener("message", event => handler(event.data));
    }

    public onceMessage(handler: Function): void {
        if (this.ipcRenderer) this.ipcRenderer.once(RelayService.CHANNEL, (event: any, message: any) => handler(message));
        else window.top.addEventListener("message", event => handler(event.data), { once: true });
    }

    public sendMessage(message: any): void {
        if (this.ipcRenderer) this.ipcRenderer.send(RelayService.CHANNEL, message);
        else window.top.postMessage(message, '*');
    }
}

export namespace RelayService {
    export const CHANNEL: string = "relay";
}
