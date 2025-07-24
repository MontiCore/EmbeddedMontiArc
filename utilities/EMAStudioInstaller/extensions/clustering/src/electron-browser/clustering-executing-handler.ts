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
import { DisposableCollection } from "@theia/core/lib/common";
import { ApplicationShell, Endpoint } from "@theia/core/lib/browser";
import { WidgetManager } from "@theia/core/lib/browser";
import { MiniBrowser, MiniBrowserProps } from "@theia/mini-browser/lib/browser/mini-browser";
import { CLUSTERING_STATIC_PATH } from "../common";

import WidgetOptions = ApplicationShell.WidgetOptions;

@injectable()
export class ClusteringExecutingHandler implements ExecutingHandler {
    @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService;
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;

    protected readonly toDispose: DisposableCollection;

    public constructor() {
        this.toDispose = new DisposableCollection();
    }

    public async isEnabled(): Promise<boolean> {
        const roots = await this.workspaceService.roots;
        const workspace = roots[0];

        return workspace && workspace.uri.endsWith("Clustering");
    }

    public async execute(): Promise<void> {
        const process = await this.scriptsService.execute({
            label: "Clustering",
            plugin: "executing",
            script: EXECUTE_SCRIPT
        });

        if (process) process.onExit(() => this.openMiniBrowser());
    }

    protected async openMiniBrowser(): Promise<void> {
        const endpoint = new Endpoint({ path: CLUSTERING_STATIC_PATH });
        const url = endpoint.getRestUrl().toString(true);
        const options = <MiniBrowserProps>{ startPage: url, toolbar: "read-only", name: "ClusterFiddle" };
        const widget = await this.widgetManager.getOrCreateWidget(MiniBrowser.Factory.ID, options);
        const addOptions = <WidgetOptions>{ area: "main", mode: "tab-after" };

        this.shell.addWidget(widget, addOptions);
        this.shell.activateWidget(widget.id);
    }
}
