/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ApplicationServer } from "@embeddedmontiarc/sol-external-core/lib/common";
import { NotificationsService } from "@embeddedmontiarc/sol-external-core/lib/main/messages";
import { StorageService } from "@embeddedmontiarc/sol-external-core/lib/main/storage/storage-service";
import { DockerRunService } from "@embeddedmontiarc/sol-external-docker/lib/main/docker-run-service";
import { DockerPhase } from "@embeddedmontiarc/sol-external-docker/lib/main/docker-phase";
import { PreparationContribution, PreparationRegistry } from "@embeddedmontiarc/sol-external-preparation/lib/main";
import { MessageType } from "@theia/core/lib/common/message-service-protocol";

import * as EventEmitter from "eventemitter3";
import * as fs from "fs-extra";
import { inject, injectable } from "inversify";
import { Workspace, WorkspaceServer } from "../common";
import { WorkspacePhase } from "./workspace-phase";

@injectable()
export class WorkspaceServerImpl extends EventEmitter implements WorkspaceServer, PreparationContribution {
    @inject(StorageService) protected readonly storage: StorageService;
    @inject(NotificationsService) protected readonly notifications: NotificationsService;
    @inject(ApplicationServer) protected readonly server: ApplicationServer;
    @inject(WorkspacePhase) protected readonly workspacePhase: WorkspacePhase;
    @inject(DockerPhase) protected readonly dockerPhase: DockerPhase;
    @inject(DockerRunService) protected readonly runner: DockerRunService;

    public static readonly KEY: string = "workspaces";

    protected workspaces: Workspace[];

    public async open(workspace: Workspace): Promise<void> {
        this.emit("open", workspace);
        await this.runner.kill();
        await this.notifications.showMessage(`Opening workspace '${workspace.path}'...`);
        return this.server.gotoPhase(this.dockerPhase.id, workspace.path);
    }

    public async close(): Promise<void> {
        this.emit("close");
        return this.server.gotoPhase(this.workspacePhase.id);
    }

    public async getWorkspaces(): Promise<Workspace[]> {
        return this.workspaces;
    }

    public async addWorkspace(workspace: Workspace): Promise<void> {
        const index = this.workspaces.findIndex(w => w.path === workspace.path);

        if (index > -1) return this.notifications.showMessage("This workspace already exists.", MessageType.Warning);

        this.workspaces.push(workspace);
        await this.save();
        this.emit("add", workspace);
    }

    public async removeWorkspace(path: string): Promise<void> {
        const index = this.workspaces.findIndex(workspace => workspace.path === path);

        if (index === -1) throw new Error(`There is no workspace with path ${path}.`);

        this.workspaces.splice(index, 1);
        await this.save();
        this.emit("remove", path);
    }

    protected async load(): Promise<void> {
        await this.notifications.showMessage("Loading workspaces...");

        const workspaces = await this.storage.getData<Workspace[]>(WorkspaceServerImpl.KEY, []);

        this.workspaces = workspaces || [];

        return this.notifications.showMessage("Workspaces loaded.");
    }

    protected async save(): Promise<void> {
        return this.storage.setData<Workspace[]>(WorkspaceServerImpl.KEY, this.workspaces);
    }

    protected async sanitize(): Promise<void> {
        await this.notifications.showMessage("Sanitizing workspaces...");

        const promises = this.workspaces.map(workspace => fs.pathExists(workspace.path));
        const bits = await Promise.all(promises);

        this.workspaces = this.workspaces.filter(() => bits.shift());

        await this.save();
        return this.notifications.showMessage("Workspaces sanitized.");
    }

    public registerPreparations(registry: PreparationRegistry): void {
        registry.registerPreparation({
            id: "workspace.load",
            priority: 40100,
            prepare: () => this.load()
        });

        registry.registerPreparation({
            id: "workspace.sanitize",
            priority: 40000,
            prepare: () => this.sanitize()
        });
    }
}
