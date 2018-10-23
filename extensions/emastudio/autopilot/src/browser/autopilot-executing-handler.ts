/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { ExecutingHandler } from "@emastudio/executing/lib/browser";
import { EXECUTE_SCRIPT } from "@emastudio/executing/lib/common";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { ScriptsService } from "@emastudio/scripts/lib/browser";
import { Process } from "@emastudio/process/lib/browser";
import { DisposableCollection } from "@theia/core/lib/common";
import { ApplicationShell } from "@theia/core/lib/browser";
import { WidgetManager } from "@theia/core/lib/browser";
import { MiniBrowser, MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { EXECUTE_KILL_SCRIPT } from "../common";
import { Deferred } from "@theia/core/lib/common/promise-util";

import WidgetOptions = ApplicationShell.WidgetOptions;

@injectable()
export class AutoPilotExecutingHandler implements ExecutingHandler {
    @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService;
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;

    protected _isRunning: boolean;
    protected _killPromise: Deferred<void>;

    protected readonly toDispose: DisposableCollection;

    public constructor() {
        this.toDispose = new DisposableCollection();
        this._isRunning = false;
    }

    public async isEnabled(): Promise<boolean> {
        const roots = await this.workspaceService.roots;
        const workspace = roots[0];

        return workspace && workspace.uri.endsWith("AutoPilot");
    }

    public get isRunning(): boolean {
        return this._isRunning;
    }

    public async execute(): Promise<void> {
        if (this.isRunning) await this.executeKill();

        return this.executeStart();
    }

    public async executeKill(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Kill Simulation",
            plugin: "executing",
            script: EXECUTE_KILL_SCRIPT
        });

        this._killPromise = new Deferred<void>();

        if (process) this.bindKillProcess(process);

        return this._killPromise.promise;
    }

    protected async executeStart(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Simulation",
            plugin: "executing",
            script: EXECUTE_SCRIPT
        });

        if (process) this.bindStartProcess(process);
    }

    protected bindKillProcess(process: Process): void {
        const event = process.onExit(this.onKillProcessExit.bind(this));

        this.toDispose.push(event);
    }

    protected bindStartProcess(process: Process): void {
        const event = process.onOutput(this.onStartProcessData.bind(this));

        this.toDispose.push(event);
    }

    protected async onKillProcessExit(): Promise<void> {
        this._isRunning = false;

        this._killPromise.resolve();
    }

    protected async onStartProcessData(data: string): Promise<void> {
        if (data.indexOf("World is initialling") > -1) {
            this._isRunning = true;

            this.toDispose.dispose();
            return this.openMiniBrowser();
        }
    }

    protected async openMiniBrowser(): Promise<void> {
        const options = <MiniBrowserProps>{ startPage: "http://localhost:8080", toolbar: "read-only", name: "AutoPilot" };
        const widget = await this.widgetManager.getOrCreateWidget(MiniBrowser.Factory.ID, options);
        const addOptions = <WidgetOptions>{ area: "main", mode: "tab-after" };

        this.shell.addWidget(widget, addOptions);
        this.shell.activateWidget(widget.id);
    }
}
