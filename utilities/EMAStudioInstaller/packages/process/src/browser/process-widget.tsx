/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject, postConstruct } from "inversify";
import { ReactWidget } from "@theia/core/lib/browser/widgets/react-widget";
import { Process } from "./process";
import { ProcessManager } from "./process-manager";
import { ReactNode, Fragment, ChangeEvent } from "react";
import { Message } from "@theia/core/lib/browser";
import { fit } from "xterm/lib/addons/fit/fit";
import { Widget } from "@theia/core/lib/browser";
import { ProcessWidgetState } from "./process-widget-state";
import { ApplicationShell } from "@theia/core/lib/browser";

import * as React from "react";

import ResizeMessage = Widget.ResizeMessage;

@injectable()
export class ProcessWidget extends ReactWidget {
    @inject(ProcessManager)
    protected readonly processManager: ProcessManager;

    @inject(ProcessWidgetState)
    protected readonly state: ProcessWidgetState;

    @inject(ApplicationShell)
    protected readonly shell: ApplicationShell;

    protected canUpdate: boolean;
    protected needsUpdate: boolean;

    public constructor() {
        super();
        this.id = ProcessWidget.WIDGET_ID;
        this.title.label = "Processes";
        this.title.iconClass = "fa fa-flag";
        this.title.closable = true;
        this.canUpdate = false;
        this.needsUpdate = true;
    }

    @postConstruct()
    protected init(): void {
        this.state.onChange(this.onStateChange.bind(this));
        this.update();
    }

    protected onStateChange(): void {
        this.shell.expandPanel("bottom");
        this.shell.activateWidget(this.id);
        this.update();
    }

    protected getSelectValue(): string {
        const selection = this.state.getSelection();

        return selection !== undefined ? `${selection}` : "<No Processes>";
    }

    protected onSelectChange(event: ChangeEvent): void {
        const target = event.target as HTMLSelectElement;
        const id = parseInt(target.value, 10);

        this.state.select(id);
    }

    protected render(): ReactNode {
        if (this.canUpdate) return <Fragment>{this.renderControls()}{this.renderProcessContents()}</Fragment>;
        else this.needsUpdate = true;
    }

    protected renderControls(): ReactNode {
        return <div id="emastudio-process-controls">
            {this.renderProcessSelector()}
            {this.renderTrash()}
            {this.renderCross()}
        </div>;
    }

    protected renderTrash(): ReactNode {
        return <i className={"fa fa-trash emastudio-process-icon " + (this.isTrashEnabled() ? '' : "fa-disabled")}
                  aria-hidden="true"
                  onClick={this.onTrashClick.bind(this)}/>;
    }

    protected isTrashEnabled(): boolean {
        const process = this.state.getSelectionAsProcess();

        return process !== undefined && process.killed;
    }

    protected async onTrashClick(): Promise<void> {
        const process = this.state.getSelectionAsProcess();
        const condition = this.isTrashEnabled();

        if (condition) return this.processManager.unregister(process!);
    }

    protected renderCross(): ReactNode {
        return <i className={"fa fa-times emastudio-process-icon " + (this.isCrossEnabled() ? '' : "fa-disabled")}
                  aria-hidden="true"
                  onClick={this.onCrossClick.bind(this)}/>;
    }

    protected isCrossEnabled(): boolean {
        const process = this.state.getSelectionAsProcess();

        return process !== undefined && !process.killed;
    }

    protected async onCrossClick(): Promise<void> {
        const process = this.state.getSelectionAsProcess();
        const condition = this.isCrossEnabled();

        if (condition) return process!.kill();
    }

    protected renderProcessSelector(): ReactNode {
        const processes = this.processManager.getProcesses();

        return <select id={ProcessWidget.SELECT_ID} value={this.getSelectValue()}
                       onChange={this.onSelectChange.bind(this)}>
            {processes.length > 0 ? this.renderSelectorOptions(processes) : this.renderDefaultSelectorOption()}
        </select>;
    }

    protected renderDefaultSelectorOption(): ReactNode {
        return <option key="No Processes" value="No Processes">No Processes</option>;
    }

    protected renderSelectorOptions(processes: Process[]): ReactNode[] {
        const nodes: ReactNode[] = [];

        for (const process of processes) {
            nodes.push(this.renderSelectorOption(process));
        }

        return nodes;
    }

    protected renderSelectorOption(process: Process): ReactNode {
        return <option value={process.id} key={process.id}>{process.label}</option>;
    }

    protected renderProcessContents(): ReactNode {
        const process = this.state.getSelectionAsProcess();
        const key = process ? this.getKey(process) : "-1";

        return <div id={ProcessWidget.CONTAINER_ID} key={key}/>;
    }

    protected getKey(process: Process): string {
        const state = process.killed ? "Terminated" : "Running";

        return `${process.id}:${state}`;
    }

    protected onUpdateRequest(message: Message): void {
        super.onUpdateRequest(message);

        const terminal = this.state.getSelectionAsTerminal();
        const node = document.getElementById(ProcessWidget.CONTAINER_ID);

        if (terminal && node) {
            terminal.open(node);
            this.resizeTerminal();
        }
    }

    protected onAfterShow(message: Message): void {
        this.canUpdate = true;

        if (this.needsUpdate) {
            this.needsUpdate = false;
            this.update();
        }
    }

    protected onBeforeHide(message: Message): void {
        this.canUpdate = false;
    }

    protected resizeTerminal(): void {
        const terminal = this.state.getSelectionAsTerminal();

        if (terminal && terminal.element) fit(terminal);
    }

    protected onResize(message: ResizeMessage): void {
        super.onResize(message);

        this.resizeTerminal();
    }
}

export namespace ProcessWidget {
    export const WIDGET_ID: string = "emastudio-process-widget";
    export const SELECT_ID: string = "emastudio-process-select";
    export const CONTAINER_ID: string = "emastudio-process-container";
}
