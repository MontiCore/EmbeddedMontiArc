/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { ApplicationClient } from "../common";
import { RouterFragment } from "./router-fragment";

import * as React from "react";
import * as ReactDOM from "react-dom";
import * as EventEmitter from "eventemitter3";

export const DEFAULT_LOGO: string = require("../../src/renderer/images/logo.png");

export const Application = Symbol("Application");
/**
 * An interface to be implemented by classes representing the renderer application.
 */
export interface Application {
    /**
     * Sets the path to the logo to be used in the application.
     */
    setLogo(logo: string): void;

    /**
     * Gets the path of the logo to be used in the application.
     */
    getLogo(): string;

    /**
     * Starts the application by rendering the content.
     */
    render(): Promise<void>;
}

@injectable()
export class RendererApplication extends EventEmitter implements Application, ApplicationClient {
    protected logo: string | undefined;

    public setLogo(logo: string): void {
        this.logo = logo;
    }

    public getLogo(): string {
        return this.logo || DEFAULT_LOGO;
    }

    public async render(): Promise<void> {
        await new Promise(resolve => ReactDOM.render(<RouterFragment/>, document.body, resolve));

        this.emit("ready");
    }
}
