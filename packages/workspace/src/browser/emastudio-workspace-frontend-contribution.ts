/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { BrowserWorkspaceFrontendContribution } from "@elysium/workspace/lib/browser/workspace-frontend-contribution";
import { inject, injectable } from "inversify";
import { OpenerService, StorageService, LabelProvider } from '@theia/core/lib/browser';
import { FileDialogService} from '@theia/filesystem/lib/browser';
import { FileSystem } from '@theia/filesystem/lib/common';
import { WorkspacePreferences, WorkspaceService } from "@theia/workspace/lib/browser";
import { ConfirmDialog } from "@theia/core/lib/browser";
import { QuickOpenWorkspace } from "@theia/workspace/lib/browser/quick-open-workspace";

@injectable()
export class EMAStudioWorkspaceFrontendContribution extends BrowserWorkspaceFrontendContribution {
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
            fileSystem, openerService, workspaceService, workspaceStorage, labelProvider,
            quickOpenWorkspace, fileDialogService, preferences
        );
    }

    public handleCommand(): void {
        const dialog = new ConfirmDialog({
            title: "Close Workspace",
            msg: "Do you really want to close the workspace?"
        });

        dialog.open().then(confirmation => {
            if (confirmation) this.workspaceService.close();
        });
    }
}
