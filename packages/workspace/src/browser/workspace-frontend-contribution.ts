/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable } from "inversify";
import { CommandContribution, CommandRegistry, MenuContribution } from "@theia/core/lib/common";
import { OpenerService, StorageService, LabelProvider } from '@theia/core/lib/browser';
import { FileDialogFactory } from '@theia/filesystem/lib/browser';
import { FileSystem } from '@theia/filesystem/lib/common';
import { WorkspaceCommands } from "@theia/workspace/lib/browser/workspace-commands";
import { WorkspaceFrontendContribution, WorkspaceService } from "@theia/workspace/lib/browser";
import { ExtendedWindowService } from "@elysium/core/lib/browser/window";
import URI from "@theia/core/lib/common/uri";

@injectable()
export class BrowserWorkspaceFrontendContribution extends WorkspaceFrontendContribution implements CommandContribution, MenuContribution {
    public constructor(
        @inject(FileSystem) protected readonly fileSystem: FileSystem,
        @inject(FileDialogFactory) protected readonly fileDialogFactory: FileDialogFactory,
        @inject(OpenerService) protected readonly openerService: OpenerService,
        @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService,
        @inject(StorageService) protected readonly workspaceStorage: StorageService,
        @inject(LabelProvider) protected readonly labelProvider: LabelProvider,
        @inject(ExtendedWindowService) protected readonly windowService: ExtendedWindowService
    ) {
        super(fileSystem, fileDialogFactory, openerService, workspaceService, workspaceStorage, labelProvider);
    }

    public registerCommands(commands: CommandRegistry): void {
        commands.registerCommand(WorkspaceCommands.OPEN, {
            isEnabled: () => true,
            execute: () => this.handleCommand()
        });
        commands.registerCommand(WorkspaceCommands.CLOSE, {
            isEnabled: () => this.workspaceService.opened,
            execute: () => this.handleCommand()
        });
    }

    public handleCommand(): void {
        const uri = new URI(window.location.href).withoutQuery();
        const url = uri.toString();

        this.windowService.redirectWindow(url);
    }
}
