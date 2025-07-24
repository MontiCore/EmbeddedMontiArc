/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { BaseDashboardModel } from "../dashboard-model";
import { FileUri } from "@theia/core/lib/node/file-uri";
import { FileSystem } from "@theia/filesystem/lib/common";
import { ExtendedWindowService } from "@elysium/core/lib/browser/window/window-service";
import { FileSystemDashboardQueue } from "./filesystem-dashboard-queue";

@injectable()
export class FileSystemDashboardModel extends BaseDashboardModel {
    @inject(FileSystem) protected readonly fileSystem: FileSystem;
    @inject(ExtendedWindowService) protected readonly windowService: ExtendedWindowService;
    @inject(FileSystemDashboardQueue) protected readonly queue: FileSystemDashboardQueue;

    public open(uri: string): void {
        const fsPath = FileUri.fsPath(uri);

        this.windowService.redirectWindow(`?root=${fsPath}`);
    }

    public async create(name: string): Promise<void> {
        const uri = FileUri.create(`/${name}`).toString();

        await this.fileSystem.createFolder(uri);
        await this.refresh();
    }

    public async delete(uri: string): Promise<void> {
        this.queue.addToQueue(uri);
        await this.refresh();

        await this.fileSystem.delete(uri);
        this.clearDiagnostics(uri);

        this.queue.removeFromQueue(uri);
        await this.refresh();
    }

    protected clearDiagnostics(uri: string): void {
        const length = localStorage.length;

        for (let i = 0; i < length; i++) {
            const key = window.localStorage.key(i);

            if (key && key.indexOf(uri) > -1) {
                window.localStorage.removeItem(key);
                i--;
            }
        }
    }
}
