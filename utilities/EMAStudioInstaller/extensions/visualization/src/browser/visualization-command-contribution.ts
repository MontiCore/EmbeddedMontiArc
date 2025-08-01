/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { CommandContribution, CommandRegistry, Command } from "@theia/core/lib/common";
import { VisualizationConditions } from "./visualization-conditions";
import { ScriptsService } from "@emastudio/scripts/lib/browser";
import { VISUALIZE_SCRIPT } from "../common";
import { ApplicationShell, Endpoint } from "@theia/core/lib/browser";
import { WidgetManager } from "@theia/core/lib/browser";
import { MiniBrowser, MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { VISUALIZATION_STATIC_PATH } from "../common";
import { IProcessExitEvent } from "@emastudio/process/lib/common";

import WidgetOptions = ApplicationShell.WidgetOptions;

export const VISUALIZATION_ICON_CLASS: string = "emastudio-visualize-icon";

export namespace VisualizationCommands {
    export const VISUALIZE: Command = {
        id: "emastudio.visualization.visualize",
        label: "Visualize Workspace",
        iconClass: VISUALIZATION_ICON_CLASS
    };
}

@injectable()
export class VisualizationCommandContribution implements CommandContribution {
    @inject(VisualizationConditions) protected conditions: VisualizationConditions;
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;

    public async registerCommands(commands: CommandRegistry): Promise<void> {
        const command = commands.registerCommand(VisualizationCommands.VISUALIZE, {
            execute: this.executeVisualize.bind(this)
        });

        return this.conditions.check(command);
    }

    protected async executeVisualize(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Visualization",
            plugin: "visualization",
            script: VISUALIZE_SCRIPT
        });

        if (process) process.onExit(this.onVisualizeProcessExit.bind(this));
    }

    protected async onVisualizeProcessExit(event: IProcessExitEvent): Promise<void> {
        if (event.code === 0) return this.openMiniBrowser();
    }

    protected async openMiniBrowser(): Promise<void> {
        const endpoint = new Endpoint({ path: VISUALIZATION_STATIC_PATH });
        const url = endpoint.getRestUrl().toString();
        const options = <MiniBrowserProps>{ startPage: url, toolbar: "read-only", name: "Visualization" };
        const widget = await this.widgetManager.getOrCreateWidget(MiniBrowser.Factory.ID, options);
        const addOptions = <WidgetOptions>{ area: "main", mode: "split-right" };

        this.shell.addWidget(widget, addOptions);
    }
}
