/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable, postConstruct } from "inversify";
import { WorkspaceService } from "@theia/workspace/lib/browser";
import { FileSystem } from "@theia/filesystem/lib/common";
import { PathsServer } from "@emastudio/paths/lib/common";
import { SCRIPTS_PATH_ID } from "../common";
import { Process, ProcessService } from "@emastudio/process/lib/browser";
import { FileUri } from "@theia/core/lib/node/file-uri";
import { ProcessType } from "@emastudio/process/lib/common";
import { isWindows } from "@theia/core/lib/common";

import URI from "@theia/core/lib/common/uri";

export interface ScriptDescription {
    readonly label: string;
    readonly plugin: string;
    readonly script: string;
    readonly args?: string[];
}

@injectable()
export class ScriptsService {
    @inject(WorkspaceService) protected readonly workspaceService: WorkspaceService;
    @inject(PathsServer) protected readonly pathsServer: PathsServer;
    @inject(FileSystem) protected readonly fileSystem: FileSystem;
    @inject(ProcessService) protected readonly processService: ProcessService;

    protected readonly processes: Map<string, Process>;
    protected workspace: URI;

    public constructor() {
        this.processes = new Map();
    }

    @postConstruct()
    protected async init(): Promise<void> {
        const roots = await this.workspaceService.roots;
        const workspace = roots[0];

        if (workspace) this.workspace = new URI(workspace.uri);
    }

    public async execute(description: ScriptDescription): Promise<Process | undefined> {
        /*const process = this.processes.get(label);

        if (process && !process.killed) return process;
        else */return this.doExecute(description);
    }

    protected async doExecute(description: ScriptDescription): Promise<Process | undefined> {
        const command = await this.getScriptCommand(description);
        const processArgs = description.args || [];
        const options = await this.getOptions(description);
        const process = await this.spawn(description.label, command, processArgs, options);

        if (process) this.processes.set(description.label, process);

        return process;
    }

    protected async getOptions(description: ScriptDescription): Promise<object> {
        const scripts = await this.getScriptsFolderAsURI();
        const cwd = scripts.resolve(this.workspace.displayName).resolve(description.plugin);

        return { cwd: FileUri.fsPath(cwd) };
    }

    protected async spawn(label: string, command: string, args: string[], options: object): Promise<Process | undefined> {
        return this.processService.spawn(label, {
            type: ProcessType.Raw,
            options: { command, args, options }
        });
    }

    protected async getScriptsFolderAsURI(): Promise<URI> {
        const scripts = await this.pathsServer.getPath(SCRIPTS_PATH_ID);

        return new URI(scripts);
    }

    protected async getScriptAsURI(description: ScriptDescription): Promise<URI> {
        const scripts = await this.getScriptsFolderAsURI();

        return scripts.resolve(this.workspace.displayName).resolve(description.plugin).resolve(description.script);
    }

    protected async getScriptCommand(description: ScriptDescription): Promise<string> {
        const scriptURI = await this.getScriptAsURI(description);
        const command = FileUri.fsPath(scriptURI);

        return isWindows ? command.replace(/\//g, "\\") : command;
    }
}
