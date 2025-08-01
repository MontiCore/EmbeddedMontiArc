/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { Disposable } from "@theia/core/lib/common";
import { PACMAN_PATH_ID } from "../common";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { WidgetManager } from "@theia/core/lib/browser";
import { ApplicationShell } from "@theia/core/lib/browser/shell";
import { PathsServer } from "@emastudio/paths/lib/common";
import { FileSystem } from "@theia/filesystem/lib/common";

@injectable()
export class PacManConditions {
    @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService;
    @inject(WidgetManager) protected readonly widgetManager: WidgetManager;
    @inject(ApplicationShell) protected readonly shell: ApplicationShell;
    @inject(PathsServer) protected readonly pathsServer: PathsServer;
    @inject(FileSystem) protected readonly fileSystem: FileSystem;

    public async check(disposable: Disposable): Promise<void> {
        const isPacManWorkspace = await this.isPacManWorkspace();
        const existsPacManFolder = await this.existsPacManFolder();

        if (!isPacManWorkspace || !existsPacManFolder) disposable.dispose();
    }

    protected async isPacManWorkspace(): Promise<boolean> {
        const roots = await this.workspaceService.roots;
        const stat = roots[0];

        return stat && stat.uri.toLowerCase().endsWith("pacman");
    }

    protected async existsPacManFolder(): Promise<boolean> {
        const uri = await this.pathsServer.getPath(PACMAN_PATH_ID);

        return this.fileSystem.exists(uri);
    }
}
