/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { bind } from "helpful-decorators";
import { BrowserWindow, BrowserWindowConstructorOptions, screen, LoadURLOptions, app, Event } from "electron";

export const WindowService = Symbol("WindowService");
/**
 * An interface to be implemented by classes which implement the necessary functionality to create and manage windows.
 */
export interface WindowService {
    /**
     * Creates a new window with given options.
     * @param options The options to be passed to the newly created window.
     * @return The newly created window instance.
     */
    create(options?: BrowserWindowConstructorOptions): BrowserWindow;

    /**
     * Creates a new main window and replaces the old one if present.
     * @param options The options to be passed to the newly created main window.
     * @return The newly created main window.
     */
    switch(options?: BrowserWindowConstructorOptions): BrowserWindow;

    /**
     * Returns the window with a given id.
     * @param id The id of the window to be fetched.
     * @return The window with the given id or undefined otherwise.
     */
    get(id: number): BrowserWindow | undefined;

    /**
     * Returns all windows created via the manager.
     * @return All the windows created via the [[WindowService]].
     */
    getAll(): BrowserWindow[];

    /**
     * Fetches the current main window if present.
     * @return The current main window if present, undefined otherwise.
     */
    getMainWindow(): BrowserWindow | undefined;

    /**
     * Shorthand method to load a url with given options into the current main window if present.
     * @param url The url which should be loaded into the current main window.
     * @param options The options to be passed to the loading procedure.
     */
    loadURL(url: string, options?: LoadURLOptions): Promise<void>;
}

@injectable()
export class WindowServiceImpl implements WindowService {
    protected readonly windows: Map<number, BrowserWindow>;

    protected mainWindow: BrowserWindow | undefined;

    public constructor() {
        this.windows = new Map();
    }

    public create(options?: BrowserWindowConstructorOptions): BrowserWindow {
        const window = options ? this.createNormalized(options) : new BrowserWindow();

        if (this.windows.size === 0) this.mainWindow = window;

        this.bindEventHandlers(window);
        this.windows.set(window.id, window);
        return window;
    }

    public switch(options?: BrowserWindowConstructorOptions): BrowserWindow {
        const mainWindow = this.mainWindow;
        const newWindow = this.create(options);

        if (mainWindow) {
            mainWindow.hide();
            newWindow.once("close", event => this.onWindowClose(event, newWindow));
            newWindow.once("ready-to-show", () => this.onWindowSwitch(mainWindow, newWindow));
        }

        return newWindow;
    }

    public get(id: number): BrowserWindow | undefined {
        return this.windows.get(id);
    }

    public getAll(): BrowserWindow[] {
        return Array.from(this.windows.values());
    }

    public getMainWindow(): BrowserWindow | undefined {
        return this.mainWindow;
    }

    public async loadURL(url: string, options?: LoadURLOptions): Promise<void> {
        const window = this.mainWindow;

        if (window) return window.loadURL(url, options);
    }

    protected createNormalized(options: BrowserWindowConstructorOptions): BrowserWindow {
        const display = screen.getPrimaryDisplay();
        const maxWidth = display.size.width;
        const maxHeight = display.size.height;

        if (options.width) options.width = Math.min(options.width, maxWidth);
        if (options.height) options.height = Math.min(options.height, maxHeight);

        return new BrowserWindow(options);
    }

    protected bindEventHandlers(window: BrowserWindow): void {
        const id = window.id;

        window.once("ready-to-show", () => window.show());
        window.once("page-title-updated", event => event.preventDefault());
        window.once("closed", () => this.windows.delete(id));
    }

    @bind
    protected onWindowFocus(): void {}

    @bind
    protected onWindowClose(event: Event, window: BrowserWindow): void {
        event.preventDefault();
        window.setClosable(false);
        app.quit();
    }

    @bind
    protected onReadyToShow(window: BrowserWindow): void {}

    @bind
    protected onWindowSwitch(oldWindow: BrowserWindow, newWindow: BrowserWindow): void {
        this.mainWindow = newWindow;

        oldWindow.destroy();
    }
}
