/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { FileStat, FileSystem } from "@theia/filesystem/lib/common";
import { PathsServer } from "@emastudio/paths/lib/common";
import { SCRIPTS_PATH_ID } from "../common";

import URI from "@theia/core/lib/common/uri";

@injectable()
export class ScriptsConditions {
    @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService;
    @inject(PathsServer) protected readonly pathsServer: PathsServer;
    @inject(FileSystem) protected readonly fileSystem: FileSystem;

    public async check(plugin: string, script: string): Promise<boolean> {
        return this.existsScript(plugin, script);
    }

    protected async existsScript(plugin: string, script: string): Promise<boolean> {
        const roots = await this.workspaceService.roots;
        const workspace = roots[0];

        if (workspace) return this.doExistsScript(workspace, plugin, script);
        else return false;
    }

    protected async doExistsScript(workspace: FileStat, plugin: string, script: string): Promise<boolean> {
        const scripts = await this.pathsServer.getPath(SCRIPTS_PATH_ID);
        const scriptsURI = new URI(scripts);
        const workspaceURI = new URI(workspace.uri);
        const uri = scriptsURI.resolve(workspaceURI.displayName).resolve(plugin).resolve(script).toString();

        return this.fileSystem.exists(uri);
    }
}
