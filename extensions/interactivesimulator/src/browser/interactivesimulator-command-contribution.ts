/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { CommandContribution, CommandRegistry, Command } from "@theia/core/lib/common";
import { InteractiveSimulatorConditions } from "./interactivesimulator-conditions";
import { ScriptsService } from "@emastudio/scripts/lib/browser";
import { InteractiveSimulatorScripts, InteractiveSimulatorStaticPaths } from "../common";
import { ApplicationShell, Endpoint, WidgetManager } from "@theia/core/lib/browser";
import { MiniBrowser, MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { WorkspaceService } from "@theia/workspace/lib/browser";

import URI from "@theia/core/lib/common/uri";

import WidgetOptions = ApplicationShell.WidgetOptions;

export namespace InteractiveSimulatorIcons {
    export const DEBUG: string = "emastudio-interactivesimulator-debug-icon";
}

export namespace InteractiveSimulatorCommands {
    export const DEBUG: Command = {
        id: "emastudio.interactivesimulator.debug",
        label: "Debug Model",
        iconClass: InteractiveSimulatorIcons.DEBUG
    };

    export const DEBUG_WOSVG: Command = {
        id: "emastudio.interactivesimulator.debug.wosvg",
        label: "Debug Model (Without SVG)",
        iconClass: InteractiveSimulatorIcons.DEBUG
    };
}

@injectable()
export class InteractiveSimulatorCommandContribution implements CommandContribution {
    @inject(InteractiveSimulatorConditions) protected readonly conditions: InteractiveSimulatorConditions;
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;
    @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService;

    public async registerCommands(registry: CommandRegistry): Promise<void> {
        await Promise.all([
            this.registerCommandDebug(registry),
            this.registerCommandDebugWosvg(registry)
        ]);
    }

    protected async registerCommandDebug(registry: CommandRegistry): Promise<void> {
        const command = registry.registerCommand(InteractiveSimulatorCommands.DEBUG, {
            execute: this.executeDebug.bind(this)
        });

        return this.conditions.checkDebug(command);
    }

    protected async registerCommandDebugWosvg(registry: CommandRegistry): Promise<void> {
        const command = registry.registerCommand(InteractiveSimulatorCommands.DEBUG_WOSVG, {
            execute: this.executeDebugWosvg.bind(this)
        });

        return this.conditions.checkDebugWosvg(command);
    }

    protected async executeDebug(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Debug Model",
            plugin: "interactivesimulator",
            script: InteractiveSimulatorScripts.DEBUG
        });

        if (process) process.onExit(() => this.openMiniBrowser());
    }

    protected async executeDebugWosvg(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Debug Model (Without SVG)",
            plugin: "interactivesimulator",
            script: InteractiveSimulatorScripts.DEBUG_WOSVG
        });

        if (process) process.onExit(() => this.openMiniBrowser());
    }

    protected async getPathFromWorkspace(): Promise<string> {
        const roots = await this.workspaceService.roots;
        const workspace = new URI(roots[0].uri);
        const name = workspace.displayName;
        const normalizedName = name.charAt(0) + name.slice(1).toLowerCase();

        return `${InteractiveSimulatorStaticPaths.INTERACTIVESIMULATOR}/html/interactive${normalizedName}.html`;
    }

    protected async openMiniBrowser(): Promise<void> {
        const path = await this.getPathFromWorkspace();
        const endpoint = new Endpoint({ path });
        const url = endpoint.getRestUrl().toString(true);
        const options = <MiniBrowserProps>{ startPage: url, toolbar: "read-only", name: "Interactive Simulator", resetBackground: true };
        const widget = await this.widgetManager.getOrCreateWidget(MiniBrowser.Factory.ID, options);
        const addOptions = <WidgetOptions>{ area: "main", mode: "tab-after" };

        this.shell.addWidget(widget, addOptions);
        this.shell.activateWidget(widget.id);
    }
}
