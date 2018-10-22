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
        const electron = windowAny.require("electron");

        this.ipcRenderer = electron.ipcRenderer;
    }

    public onMessage(handler: Function): void {
        this.ipcRenderer.on(RelayService.CHANNEL, (event: any, message: any) => handler(message));
    }

    public onceMessage(handler: Function): void {
        this.ipcRenderer.once(RelayService.CHANNEL, (event: any, message: any) => handler(message));
    }

    public sendMessage(message: any): void {
        this.ipcRenderer.send(RelayService.CHANNEL, message);
    }
}

export namespace RelayService {
    export const CHANNEL: string = "relay";
}
