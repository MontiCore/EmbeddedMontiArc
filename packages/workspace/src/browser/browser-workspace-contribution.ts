/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { Command, CommandContribution, CommandRegistry } from '@theia/core/lib/common/command';
import { Widget } from '@theia/core/lib/browser';
import { FileSystemDashboardWidget } from "@elysium/dashboard/lib/browser";

export namespace BrowserWorkspaceCommands {
    export const OPEN_DASHBOARD: Command = {
        id: "browser-workspace.open-dashboard",
        label: "Open Dashboard..."
    };
}

@injectable()
export class BrowserWorkspaceContribution implements CommandContribution {
    @inject(FileSystemDashboardWidget) protected readonly widget: FileSystemDashboardWidget;

    public registerCommands(registry: CommandRegistry): void {
        registry.registerCommand(BrowserWorkspaceCommands.OPEN_DASHBOARD, {
            execute: () => this.handleOpenDashboard()
        });
    }

    protected handleOpenDashboard(): void {
        Widget.attach(this.widget, document.body);
    }
}
