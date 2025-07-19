/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable } from "inversify";
import { CommandContribution, CommandRegistry, MenuContribution } from "@theia/core/lib/common";
import { OpenerService, StorageService, LabelProvider } from '@theia/core/lib/browser';
import { FileSystem } from '@theia/filesystem/lib/common';
import { WorkspaceCommands } from "@theia/workspace/lib/browser/workspace-commands";
import { WorkspaceFrontendContribution, WorkspaceService } from "@theia/workspace/lib/browser";
import { ExtendedWindowService } from "@elysium/core/lib/browser/window";
import { QuickOpenWorkspace } from "@theia/workspace/lib/browser/quick-open-workspace";
import { FileDialogService } from "@theia/filesystem/lib/browser";
import { WorkspacePreferences } from "@theia/workspace/lib/browser";

import URI from "@theia/core/lib/common/uri";

@injectable()
export class BrowserWorkspaceFrontendContribution extends WorkspaceFrontendContribution implements CommandContribution, MenuContribution {
    @inject(ExtendedWindowService) protected readonly windowService: ExtendedWindowService;

    public constructor(
        @inject(FileSystem) protected readonly fileSystem: FileSystem,
        @inject(OpenerService) protected readonly openerService: OpenerService,
        @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService,
        @inject(StorageService) protected readonly workspaceStorage: StorageService,
        @inject(LabelProvider) protected readonly labelProvider: LabelProvider,
        @inject(QuickOpenWorkspace) protected readonly quickOpenWorkspace: QuickOpenWorkspace,
        @inject(FileDialogService) protected readonly fileDialogService: FileDialogService,
        @inject(WorkspacePreferences) protected preferences: WorkspacePreferences
    ) {
        super(
            fileSystem, openerService, workspaceService, workspaceStorage, labelProvider, quickOpenWorkspace,
            fileDialogService, preferences
        );
    }

    // TODO: This is faulty, it should not be handleCommand everywhere.
    public registerCommands(commands: CommandRegistry): void {
        commands.registerCommand(WorkspaceCommands.OPEN, {
            isEnabled: () => true,
            execute: () => alert("Not implemented")
        });
        commands.registerCommand(WorkspaceCommands.CLOSE, {
            isEnabled: () => this.workspaceService.opened,
            execute: () => this.handleCommand()
        });
        commands.registerCommand(WorkspaceCommands.OPEN_WORKSPACE, {
            isEnabled: () => true,
            execute: () => alert("Not implemented")
        });
        commands.registerCommand(WorkspaceCommands.OPEN_RECENT_WORKSPACE, {
            isEnabled: () => this.workspaceService.hasHistory,
            execute: () => alert("Not implemented")
        });
    }

    public handleCommand(): void {
        const uri = new URI(window.location.href).withoutQuery();
        const url = uri.toString();

        this.windowService.redirectWindow(url);
    }
}
