/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { FileSystemDashboardModel } from "@elysium/dashboard/lib/browser/filesystem";
import { WorkspaceService } from "@theia/workspace/lib/browser";

import URI from "@theia/core/lib/common/uri";

@injectable()
export class EMAStudioDashboardModel extends FileSystemDashboardModel {
    @inject(WorkspaceService) protected readonly workspace: WorkspaceService;

    public open(uri: string): void {
        this.workspace.open(new URI(uri));
    }
}
