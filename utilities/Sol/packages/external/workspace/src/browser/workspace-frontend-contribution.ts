/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ConfirmDialog } from "@theia/core/lib/browser";
import { CommandRegistry, MenuModelRegistry } from "@theia/core/lib/common";
import { WorkspaceCommands, WorkspaceFrontendContribution as BaseWorkspaceFrontendContribution } from "@theia/workspace/lib/browser";
import { inject, injectable } from "inversify";
import { WorkspaceServer } from "../common";

@injectable()
export class WorkspaceFrontendContribution extends BaseWorkspaceFrontendContribution {
    @inject(WorkspaceServer) protected readonly server: WorkspaceServer;

    protected commands: CommandRegistry;

    public registerCommands(registry: CommandRegistry): void {
        this.commands = registry;

        super.registerCommands(registry);
    }

    public registerMenus(registry: MenuModelRegistry): void {
        super.registerMenus(registry);
        this.unregisterMenus(registry);
        this.unregisterCommands();
        this.doRegisterCommands();
    }

    protected unregisterCommands(): void {
        this.commands.unregisterCommand(WorkspaceCommands.OPEN);
        this.commands.unregisterCommand(WorkspaceCommands.OPEN_FILE);
        this.commands.unregisterCommand(WorkspaceCommands.OPEN_FOLDER);
        this.commands.unregisterCommand(WorkspaceCommands.OPEN_RECENT_WORKSPACE);
        this.commands.unregisterCommand(WorkspaceCommands.OPEN_WORKSPACE);
        this.commands.unregisterCommand(WorkspaceCommands.SAVE_AS);
        this.commands.unregisterCommand(WorkspaceCommands.SAVE_WORKSPACE_AS);
    }

    protected doRegisterCommands(): void {
        delete this.commands;
    }

    protected unregisterMenus(registry: MenuModelRegistry): void {
        registry.unregisterMenuAction(WorkspaceCommands.OPEN);
        registry.unregisterMenuAction(WorkspaceCommands.OPEN_FILE);
        registry.unregisterMenuAction(WorkspaceCommands.OPEN_FOLDER);
        registry.unregisterMenuAction(WorkspaceCommands.OPEN_RECENT_WORKSPACE);
        registry.unregisterMenuAction(WorkspaceCommands.OPEN_WORKSPACE);
        registry.unregisterMenuAction(WorkspaceCommands.SAVE_AS);
        registry.unregisterMenuAction(WorkspaceCommands.SAVE_WORKSPACE_AS);
    }

    protected async closeWorkspace(): Promise<void> {
        const dialog = new ConfirmDialog({
            title: WorkspaceCommands.CLOSE.label!,
            msg: 'Do you really want to close the workspace?'
        });

        if (await dialog.open()) return this.server.close();
    }
}
