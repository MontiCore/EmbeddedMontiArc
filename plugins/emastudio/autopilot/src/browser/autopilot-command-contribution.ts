/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { CommandContribution, CommandRegistry, Command } from "@theia/core/lib/common";
import { AutoPilotConditions } from "./autopilot-conditions";
import { ScriptsService } from "@emastudio/scripts/lib/browser";
import { EXECUTE_DISTRIBUTED_SCRIPT, EXECUTE_DISTRIBUTED_KILL_SCRIPT } from "../common";
import { Process } from "@emastudio/process/lib/browser";
import { ApplicationShell } from "@theia/core/lib/browser";
import { WidgetManager } from "@theia/core/lib/browser";
import { MiniBrowser, MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { Deferred } from "@theia/core/lib/common/promise-util";

import WidgetOptions = ApplicationShell.WidgetOptions;

export const AUTOPILOT_ICON_CLASS: string = "emastudio-execute-distributed-icon";

export namespace AutoPilotCommands {
    export const EXECUTE_DISTRIBUTED: Command = {
        id: "emastudio.autopilot.execute.distributed",
        label: "Execute Model (Distributed)",
        iconClass: AUTOPILOT_ICON_CLASS
    };
}

@injectable()
export class AutoPilotCommandContribution implements CommandContribution {
    @inject(AutoPilotConditions) protected readonly conditions: AutoPilotConditions;
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;

    protected _isRunning: boolean;
    protected _killPromise: Deferred<void>;
    protected _startPromise: Deferred<void>;

    public constructor() {
        this._isRunning = false;
    }

    public async registerCommands(commands: CommandRegistry): Promise<void> {
        const command = commands.registerCommand(AutoPilotCommands.EXECUTE_DISTRIBUTED, {
            execute: this.executeDistributed.bind(this)
        });

        return this.conditions.check(command);
    }

    public get isRunning(): boolean {
        return this._isRunning;
    }

    protected async executeDistributed(): Promise<void> {
        if (this.isRunning) await this.executeDistributedKill();

        return this.executeDistributedStart();
    }

    public async executeDistributedKill(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Kill Simulation",
            plugin: "executing",
            script: EXECUTE_DISTRIBUTED_KILL_SCRIPT
        });

        this._killPromise = new Deferred<void>();

        if (process) this.bindKillProcess(process);

        return this._killPromise.promise;
    }

    protected async executeDistributedStart(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Simulation (Distributed)",
            plugin: "executing",
            script: EXECUTE_DISTRIBUTED_SCRIPT
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
        window.setTimeout(async () => this.openMiniBrowser(), 10000);
    }

    protected async openMiniBrowser(): Promise<void> {
        const options = <MiniBrowserProps>{
            startPage: "http://localhost:80/visualization",
            toolbar: "read-only",
            name: "AutoPilot (Distributed)"
        };
        const widget = await this.widgetManager.getOrCreateWidget(MiniBrowser.Factory.ID, options);
        const addOptions = <WidgetOptions>{ area: "main", mode: "tab-after" };

        this.shell.addWidget(widget, addOptions);
        this.shell.activateWidget(widget.id);
    }
}
