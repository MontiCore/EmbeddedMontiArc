/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable } from "inversify";
import { FileSystemDashboardContribution } from "@elysium/dashboard/lib/browser/filesystem";
import { FileSystem } from "@theia/filesystem/lib/common";
import {
    DemosDashboardWidget, FileSystemDashboardCommands, FileSystemDashboardContextMenu, FileSystemDashboardWidget
} from "@elysium/dashboard/lib/browser";
import { MenuModelRegistry, CommandRegistry } from "@theia/core/lib/common";
import { EMAStudioDashboardModel } from "./emastudio-dashboard-model";

@injectable()
export class EMAStudioDashboardContribution extends FileSystemDashboardContribution {
    protected readonly model: EMAStudioDashboardModel;

    public constructor(
        @inject(FileSystemDashboardWidget) protected readonly widget: FileSystemDashboardWidget,
        @inject(DemosDashboardWidget) protected readonly demosWidget: DemosDashboardWidget,
        @inject(FileSystem) protected readonly fileSystem: FileSystem
    ) {
        super(widget, demosWidget, fileSystem);
    }

    public registerCommands(registry: CommandRegistry): void {
        registry.registerCommand(FileSystemDashboardCommands.OPEN_WORKSPACE, {
            execute: () => this.handleOpenWorkspace()
        });
    }

    public registerMenus(menus: MenuModelRegistry): void {
        menus.registerMenuAction(FileSystemDashboardContextMenu.OPEN, {
            commandId: FileSystemDashboardCommands.OPEN_WORKSPACE.id
        });
    }

    protected handleOpenWorkspace(): void {
        this.widget.model.open(this.model.selectedItem.uri);
    }
}
