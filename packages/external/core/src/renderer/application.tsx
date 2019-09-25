/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { ApplicationClient } from "../common";
import { RouterFragment } from "./router-fragment";

import * as React from "react";
import * as ReactDOM from "react-dom";
import * as EventEmitter from "eventemitter3";

export const Application = Symbol("Application");
/**
 * An interface to be implemented by classes representing the renderer application.
 */
export interface Application {
    /**
     * Starts the application by rendering the content.
     */
    render(): Promise<void>;
}

@injectable()
export class RendererApplication extends EventEmitter implements Application, ApplicationClient {
    public async render(): Promise<void> {
        await new Promise(resolve => ReactDOM.render(<RouterFragment/>, document.body, resolve));

        this.emit("ready");
    }
}
