/*
 * (c) https://github.com/MontiCore/monticore
 */
import { BrowserWindowConstructorOptions } from "electron";

// tslint:disable:no-any

export const WINDOW_PATH: string = "/services/window";

export const WindowServer = Symbol("WindowServer");
/**
 * An interface which is implemented by classes which handle creation and management of windows.
 */
export interface WindowServer {
    /**
     * Creates a window with the given options.
     * @param options The options to be passed to the newly created window.
     * @return The id of the newly created window.
     */
    create(options?: BrowserWindowConstructorOptions): Promise<number>;

    /**
     * Calls a given method with given arguments on the window with the given id.
     * @param id The id of the window on which the methods should be called on.
     * @param method The method to be called on the window.
     * @param args The arguments to be passed to the called method.
     */
    callOnWindow<ReturnType>(id: number, method: string, ...args: any[]): Promise<ReturnType>;
}
