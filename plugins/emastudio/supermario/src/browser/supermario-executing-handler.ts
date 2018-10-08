/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { ExecutingHandler } from "@emastudio/executing/lib/browser";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { EXECUTE_SCRIPT } from "@emastudio/executing/lib/common";
import { ScriptsService } from "@emastudio/scripts/lib/browser";
import { ApplicationShell } from "@theia/core/lib/browser";
import { WidgetManager } from "@theia/core/lib/browser";
import { MiniBrowser, MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { Endpoint } from "@theia/core/lib/browser";
import { SUPERMARIO_STATIC_PATH } from "../common";

import WidgetOptions = ApplicationShell.WidgetOptions;

@injectable()
export class SuperMarioExecutingHandler implements ExecutingHandler {
    @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService;
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;

    public async isEnabled(): Promise<boolean> {
        const roots = await this.workspaceService.roots;
        const workspace = roots[0];

        return workspace && workspace.uri.endsWith("SuperMario");
    }

    public async execute(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Simulation",
            plugin: "executing",
            script: EXECUTE_SCRIPT
        });

        if (process) process.onExit(() => this.openMiniBrowser());
    }

    protected async openMiniBrowser(): Promise<void> {
        const endpoint = new Endpoint({ path: `${SUPERMARIO_STATIC_PATH}/simulation.html` });
        const url = endpoint.getRestUrl().toString();
        const options = <MiniBrowserProps>{ startPage: url, toolbar: "read-only", name: "SuperMario" };
        const widget = await this.widgetManager.getOrCreateWidget(MiniBrowser.Factory.ID, options);
        const addOptions = <WidgetOptions>{ area: "main", mode: "tab-after" };

        this.shell.addWidget(widget, addOptions);
        this.shell.activateWidget(widget.id);
    }
}
