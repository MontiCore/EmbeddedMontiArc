/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { Command, CommandContribution, CommandRegistry } from "@theia/core/lib/common";
import { AutoPilotConditions } from "./autopilot-conditions";
import { ScriptsService } from "@emastudio/scripts/lib/browser";
import { Process } from "@emastudio/process/lib/browser";
import { ApplicationShell } from "@theia/core/lib/browser";
import { WidgetManager } from "@theia/core/lib/browser";
import { MiniBrowser, MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { Deferred } from "@theia/core/lib/common/promise-util";
import { EXECUTE_ICON_CLASS } from "@emastudio/executing/lib/browser";

import WidgetOptions = ApplicationShell.WidgetOptions;

export const AUTOPILOT_ICON_CLASS: string = "emastudio-execute-distributed-icon";

export namespace AutoPilotCommands {
    export const EXECUTE_DISTRIBUTED: Command = {
        id: "emastudio.autopilot.execute.distributed",
        label: "Execute Model (Distributed)",
        iconClass: AUTOPILOT_ICON_CLASS
    };

    export const EXECUTE_MODELICA: Command = {
        id: "emastudio.autopilot.execute.modelica",
        label: "Execute Model (Modelica)",
        iconClass: EXECUTE_ICON_CLASS
    };
}

@injectable()
export abstract class AutoPilotCommandContribution implements CommandContribution {
    @inject(AutoPilotConditions) protected readonly conditions: AutoPilotConditions;
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;

    protected _isRunning: boolean;
    protected _killPromise: Deferred<void>;
    protected _startPromise: Deferred<void>;

    protected constructor() {
        this._isRunning = false;
    }

    public abstract registerCommands(commands: CommandRegistry): void;

    public get isRunning(): boolean {
        return this._isRunning;
    }

    protected async execute(): Promise<void> {
        if (this.isRunning) await this.executeKill();

        return this.executeStart();
    }

    public abstract async executeKill(): Promise<void>;

    protected async doExecuteKill(script: string): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Kill Simulation",
            plugin: "executing",
            script: script
        });

        this._killPromise = new Deferred<void>();

        if (process) this.bindKillProcess(process);

        return this._killPromise.promise;
    }

    protected abstract async executeStart(): Promise<void>;

    protected async doExecuteStart(label: string, script: string): Promise<void> {
        const process = await this.scriptsService.execute({
            label: label,
            plugin: "executing",
            script: script
        });

        this._startPromise = new Deferred<void>();

        if (process) this.bindStartProcess(process);

        return this._startPromise.promise;
    }

    protected bindKillProcess(process: Process): void {
        process.onExit(this.onKillProcessExit.bind(this));
    }

    protected bindStartProcess(process: Process): void {
        process.onExit(this.onStartProcessExit.bind(this));
    }

    protected onKillProcessExit(): void {
        this._isRunning = false;

        this._killPromise.resolve();
    }

    protected onStartProcessExit(): void {
        this._isRunning = true;

        this._startPromise.resolve();
    }

    protected async openMiniBrowser(options: MiniBrowserProps): Promise<void> {
        const widget = await this.widgetManager.getOrCreateWidget(MiniBrowser.Factory.ID, options);
        const addOptions = <WidgetOptions>{ area: "main", mode: "tab-after" };

        this.shell.addWidget(widget, addOptions);
        this.shell.activateWidget(widget.id);
    }
}
