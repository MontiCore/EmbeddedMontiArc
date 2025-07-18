/*
 * (c) https://github.com/MontiCore/monticore
 */
/*
 * Parts taken from `terminal-widget-impl.ts` of Theia.
 */
import { CircularProgress } from "@material-ui/core";
import { Disposable } from "@theia/core/lib/common";
import { ReactWidget, Message, Widget, ConfirmDialog } from "@theia/core/lib/browser";
import { ThemeService } from "@theia/core/lib/browser/theming";
import { bind, debounce } from "helpful-decorators";
import { inject, injectable, postConstruct } from "inversify";
import { ReactNode } from "react";
import { ITheme, Terminal } from "xterm";
import { proposeGeometry } from "xterm/lib/addons/fit/fit";
import { ConfigurationManager } from "../configuration-manager";
import { ConfigurationProcessorSelectionService } from "./configuration-processor-selection-service";
import { ConfigurationProcessorService } from "./configuration-processor-service";
import { v4 } from "uuid";

import {
    BufferChangeEvent,
    BufferClearEvent, BufferRemoveEvent,
    ConfigurationProcessorBuffer
} from "./configuration-processor-buffer";

import {
    Configuration,
    ConfigurationExitEvent,
    ConfigurationProcessorWatcher
} from "../../common";

import * as React from "react";

import ResizeMessage = Widget.ResizeMessage;
import Fragment = React.Fragment;

@injectable()
export class ConfigurationProcessorWidget extends ReactWidget { // TODO: Maybe link to preferences?
    @inject(ThemeService) protected readonly theme: ThemeService;
    @inject(ConfigurationProcessorWatcher) protected readonly watcher: ConfigurationProcessorWatcher;
    @inject(ConfigurationProcessorService) protected readonly service: ConfigurationProcessorService;
    @inject(ConfigurationProcessorBuffer) protected readonly buffer: ConfigurationProcessorBuffer;
    @inject(ConfigurationManager) protected readonly manager: ConfigurationManager;

    @inject(ConfigurationProcessorSelectionService)
    protected readonly selectionService: ConfigurationProcessorSelectionService;

    protected readonly terminal: Terminal;

    protected runningConfigurations: Configuration[];
    protected bufferedConfigurations: Configuration[];
    protected isTerminalOpen: boolean;
    protected state: string;

    public constructor() {
        super();

        this.id = ConfigurationProcessorWidget.ID;
        this.title.label = "Run";
        this.title.caption = "Run";
        this.title.closable = true;
        this.title.iconClass = "fa fa-play";

        this.terminal = new Terminal({ theme: this.getTerminalTheme() });
        this.isTerminalOpen = false;
        this.node.tabIndex = -1;
    }

    @postConstruct()
    protected async init(): Promise<void> {
        this.toDispose.pushAll([
            Disposable.create(() => this.terminal.dispose()),
            this.theme.onThemeChange(this.onThemeChanged),
            this.selectionService.onSelectionChanged(this.onSelectionChanged),
            this.watcher.onConfigurationExited(this.onConfigurationExited),
            this.buffer.onChanged(this.onBufferChanged),
            this.buffer.onCleared(this.onBufferCleared),
            this.buffer.onRemoved(this.onBufferRemoved)
        ]);

        this.update();
        return this.updateConfigurations();
    }

    protected async updateConfigurations(): Promise<void> {
        this.runningConfigurations = await this.service.getRunningConfigurations();
        this.bufferedConfigurations = await this.buffer.getBufferedConfigurations();

        this.update();
    }

    protected get selection(): Configuration | undefined {
        return this.selectionService.getSelection();
    }

    protected select(selection: Configuration): void {
        this.selectionService.select(selection);
    }

    protected getTerminalTheme(): ITheme {
        const rootStyles = getComputedStyle(document.documentElement);
        const foreground = rootStyles.getPropertyValue("--theia-ui-font-color1").trim();
        const background = rootStyles.getPropertyValue("--theia-layout-color0").trim();
        const selection = rootStyles.getPropertyValue("--theia-transparent-accent-color2").trim();

        return { foreground: foreground, background: background, selection: selection, cursor: foreground };
    }

    @debounce(500)
    protected resizeTerminal(): void {
        if (this.isTerminalOpen) {
            const geometry = proposeGeometry(this.terminal);
            const condition = geometry &&
                !Number.isNaN(geometry.cols) && !Number.isNaN(geometry.rows) &&
                Number.isFinite(geometry.cols) && Number.isFinite(geometry.rows);

            if (condition) this.terminal.resize(geometry.cols, geometry.rows);
        }
    }

    protected clearTerminal(): void {
        this.terminal.reset();
        this.terminal.clear();
    }

    protected switchTerminal(configuration: Configuration | undefined): void {
        const buffer = configuration ? this.buffer.get(configuration.uuid) : undefined;

        this.clearTerminal();

        if (buffer) this.terminal.write(buffer);
    }

    protected withSelection(callback: any): any { // tslint:disable-line:no-any
        return this.selection ? callback : () => {};
    }

    protected isConfigurationRunning(configuration: Configuration | undefined): boolean {
        if (configuration) return this.runningConfigurations.findIndex(c => c.uuid === configuration.uuid) > -1;
        else return false;
    }

    protected getControlClassName(): string {
        return this.selection ? "control" : "control disabled";
    }

    public render(): ReactNode {
        return <div className="sol-runtime-configurations run-widget">
            {this.runningConfigurations && this.bufferedConfigurations ? this.doRender() : this.renderLoader()}
        </div>;
    }

    protected doRender(): ReactNode {
        return <Fragment>
            {this.renderTopRow()}
            {this.renderBottomRow()}
        </Fragment>;
    }

    public renderLoader(): ReactNode {
        return <div className="loader-container">
            <CircularProgress className="loader"/>
        </div>;
    }

    public renderMessage(message: string): ReactNode {
        return <div className="message-container">
            <span className="message">{message}</span>
        </div>;
    }

    protected renderTopRow(): ReactNode {
        return <div className="top-row">
            {this.bufferedConfigurations.map(configuration => this.renderTab(configuration))}
        </div>;
    }

    protected renderTab(configuration: Configuration): ReactNode {
        const className = this.selection && (this.selection.uuid === configuration.uuid) ? "tab selected" : "tab";

        return <div key={configuration.uuid} className={className} onClick={() => this.onTabClicked(configuration)}>
            {this.renderTabLabel(configuration)}
            {this.renderTabClose(configuration)}
        </div>;
    }

    protected renderTabLabel(configuration: Configuration): ReactNode {
        return <div className="label">
            {configuration.name}
        </div>;
    }

    protected renderTabClose(configuration: Configuration): ReactNode {
        return <div className="fa fa-close close" onClick={() => this.onTabCloseClicked(configuration)}/>;
    }

    protected renderBottomRow(): ReactNode {
        return <div className="bottom-row">
            {this.renderLeftColumn()}
            {this.renderRightColumn()}
        </div>;
    }

    protected renderLeftColumn(): ReactNode {
        return <div className="left-column">
            {this.renderPlayButton()}
            {this.renderTrashButton()}
        </div>;
    }

    protected renderPlayButton(): ReactNode {
        const className = this.getControlClassName();
        const onClick = this.withSelection(this.onPlayClicked);
        const iconClass = this.isConfigurationRunning(this.selection) ? "fa fa-stop" : "fa fa-play";

        return <div className={className} onClick={onClick}>
            <div className={iconClass}/>
        </div>;
    }

    protected renderTrashButton(): React.ReactNode {
        const className = this.getControlClassName();
        const onClick = this.withSelection(this.onTrashClicked);

        return <div className={className} onClick={onClick}>
            <div className="fa fa-trash"/>
        </div>;
    }

    protected renderRightColumn(): ReactNode {
        return <div className="right-column">
            {this.renderTerminalContainer()}
            {this.selection ? null /* tslint:disable-line:no-null-keyword */ : this.renderMessage("No Selection")}
        </div>;
    }

    protected renderTerminalContainer(): ReactNode {
        return <div key={this.state} className="terminal-container" ref={this.onTerminalContainerRendered}/>;
    }

    protected onActivateRequest(message: Message): void {
        super.onActivateRequest(message);
        this.node.focus();
    }

    protected onResize(message: ResizeMessage): void {
        super.onResize(message);
        this.resizeTerminal();
    }

    protected onAfterAttach(message: Message): void {
        this.state = v4();

        super.onAfterAttach(message);
        this.update();
    }

    @bind
    protected async onTabClicked(configuration: Configuration): Promise<void> {
        this.select(configuration);
    }

    @bind
    protected async onTabCloseClicked(configuration: Configuration): Promise<void> {
        const dialog = new ConfirmDialog({
            title: "Confirmation",
            msg: "Configuration is still running, are you sure that you want to terminate it?"
        });
        const isRunning = await this.service.isConfigurationRunning(configuration.uuid);
        const condition = isRunning && await dialog.open();

        if (condition) await this.service.killConfiguration(configuration.uuid);

        this.buffer.remove(configuration.uuid);
    }

    @bind
    protected async onPlayClicked(): Promise<void> {
        if (this.selection) {
            const isRunning = await this.service.isConfigurationRunning(this.selection.uuid);

            if (isRunning) return this.service.killConfiguration(this.selection.uuid);
            else return this.service.runConfiguration(this.selection);
        }
    }

    @bind
    protected async onTrashClicked(): Promise<void> {
        if (this.selection) this.buffer.clear(this.selection.uuid);
    }

    @bind
    protected async onThemeChanged(): Promise<void> {
        this.terminal.setOption("theme", this.getTerminalTheme());
    }

    @bind
    protected onTerminalContainerRendered(container: HTMLDivElement | null): void {
        if (container) {
            this.isTerminalOpen = true;

            this.terminal.open(container);
            this.terminal.setOption("theme", this.getTerminalTheme());
            this.switchTerminal(this.selection);
            this.resizeTerminal();
        }
    }

    @bind
    protected async onConfigurationExited(event: ConfigurationExitEvent): Promise<void> {
        return this.updateConfigurations();
    }

    @bind
    protected async onBufferChanged(event: BufferChangeEvent): Promise<void> {
        if (this.selection && this.selection.uuid === event.uuid) this.terminal.write(event.data);
    }

    @bind
    protected async onBufferCleared(event: BufferClearEvent): Promise<void> {
        if (this.selection && this.selection.uuid === event.uuid) this.clearTerminal();
    }

    @bind
    protected async onBufferRemoved(event: BufferRemoveEvent): Promise<void> {
        return this.updateConfigurations();
    }

    @bind
    protected async onSelectionChanged(configuration: Configuration | undefined): Promise<void> {
        this.switchTerminal(configuration);

        return this.updateConfigurations();
    }
}

export namespace ConfigurationProcessorWidget {
    export const ID: string = "configuration-processor";
}
