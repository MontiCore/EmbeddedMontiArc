/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import {
    MenuContribution, MenuModelRegistry,
    Command, CommandContribution, CommandRegistry
} from "@theia/core/lib/common";
import { ConfirmDialog, ConfirmDialogProps, Widget } from "@theia/core/lib/browser";
import { DemosDashboardWidget } from "../demos/demos-dashboard-widget";
import { FileSystemDashboardWidget } from "./filesystem-dashboard-widget";
import { FileSystemDashboardModel } from "./filesystem-dashboard-model";
import { AsyncSingleTextInputDialog, AsyncSingleTextInputDialogProps } from "@elysium/core/lib/browser";
import { FileUri } from "@theia/core/lib/node/file-uri";
import { FileSystem } from "@theia/filesystem/lib/common";

export const ITEM_CONTEXT_MENU = ["filesystem-dashboard-item-context-menu"];
export const CONTROL_CONTEXT_MENU = ["filesystem-dashboard-control-context-menu"];

export namespace FileSystemDashboardContextMenu {
    export const NEW = [...CONTROL_CONTEXT_MENU, "1_new"];
    export const NEW_FROM_DEMO = [...CONTROL_CONTEXT_MENU, "1_new_from_demo"];

    export const OPEN = [...ITEM_CONTEXT_MENU, "1_open"];
    export const DELETE = [...ITEM_CONTEXT_MENU, "2_delete"];
}

export namespace FileSystemDashboardCommands {
    export const NEW_WORKSPACE: Command = {
        id: "filesystem-dashboard.new-workspace",
        label: "New Workspace"
    };

    export const NEW_WORKSPACE_FROM_DEMO: Command = {
        id: "filesystem-dashboard.new-workspace-demo",
        label: "New Workspace from Demo"
    };

    export const OPEN_WORKSPACE: Command = {
        id: "filesystem-dashboard.open-workspace",
        label: "Open Workspace"
    };

    export const DELETE_WORKSPACE: Command = {
        id: "filesystem-dashboard.delete-workspace",
        label: "Delete Workspace"
    };
}

@injectable()
export class FileSystemDashboardContribution implements CommandContribution, MenuContribution {
    protected readonly model: FileSystemDashboardModel;

    public constructor(
        @inject(FileSystemDashboardWidget) protected readonly widget: FileSystemDashboardWidget,
        @inject(DemosDashboardWidget) protected readonly demosWidget: DemosDashboardWidget,
        @inject(FileSystem) protected readonly fileSystem: FileSystem
    ) {
        this.model = this.widget.model;
    }

    public registerCommands(registry: CommandRegistry): void {
        registry.registerCommand(FileSystemDashboardCommands.NEW_WORKSPACE, {
            execute: async () => await this.handleNewWorkspace()
        });

        registry.registerCommand(FileSystemDashboardCommands.NEW_WORKSPACE_FROM_DEMO, {
            execute: () => this.handleNewWorkspaceFromDemo()
        });

        registry.registerCommand(FileSystemDashboardCommands.OPEN_WORKSPACE, {
            execute: () => this.handleOpenWorkspace()
        });

        registry.registerCommand(FileSystemDashboardCommands.DELETE_WORKSPACE, {
            execute: async () => await this.handleDeleteWorkspace()
        });
    }

    public registerMenus(menus: MenuModelRegistry): void {
        menus.registerMenuAction(FileSystemDashboardContextMenu.NEW, {
            commandId: FileSystemDashboardCommands.NEW_WORKSPACE.id
        });

        menus.registerMenuAction(FileSystemDashboardContextMenu.NEW_FROM_DEMO, {
            commandId: FileSystemDashboardCommands.NEW_WORKSPACE_FROM_DEMO.id
        });

        menus.registerMenuAction(FileSystemDashboardContextMenu.OPEN, {
            commandId: FileSystemDashboardCommands.OPEN_WORKSPACE.id
        });

        menus.registerMenuAction(FileSystemDashboardContextMenu.DELETE, {
            commandId: FileSystemDashboardCommands.DELETE_WORKSPACE.id
        });
    }

    protected async handleNewWorkspace(): Promise<void> {
        const props = <AsyncSingleTextInputDialogProps> {
            title: "New Workspace",
            confirmButtonLabel: "Create",
            initialValue: "New-Workspace",
            validateAsync: this.validateWorkspaceName.bind(this)
        };
        const dialog = new AsyncSingleTextInputDialog(props);
        const name = await dialog.open();

        await this.model.create(name!);
    }

    protected async validateWorkspaceName(name: string): Promise<string> {
        const uri = FileUri.fsPath(`/${name}`).toString();

        if (name.length === 0) return "Please enter a workspace name";
        if (name.match(/[^\w\-.]+/)) return "The workspace name contains illegal characters";
        if (await this.fileSystem.exists(uri)) return "A workspace with such a name already exists";

        return '';
    }

    protected handleNewWorkspaceFromDemo(): void {
        Widget.attach(this.demosWidget, document.body);
    }

    protected handleOpenWorkspace(): void {
        this.widget.model.open(this.model.selectedItem.uri);
    }

    protected async handleDeleteWorkspace(): Promise<void> {
        const props = <ConfirmDialogProps>{
            title: "Confirmation",
            msg: "Are you sure you want to delete this workspace?",
            cancel: "No",
            ok: "Yes"
        };
        const dialog = new ConfirmDialog(props);
        const choice = await dialog.open();

        if (choice) await this.model.delete(this.model.selectedItem.uri);
    }
}
