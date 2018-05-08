/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { BaseDashboard, DashboardItem } from "../dashboard";
import { FileSystem, FileStat } from "@theia/filesystem/lib/common";
import { FileUri } from "@theia/core/lib/node/file-uri";
import URI from "@theia/core/lib/common/uri";
import { FileSystemDashboardQueue } from "./filesystem-dashboard-queue";

export const DEFAULT_ITEM_CLASS = "elysium-dashboard-item elysium-dashboard-icon-folder";
export const PROCESS_ITEM_CLASS = "elysium-dashboard-item elysium-dashboard-icon-loader";

@injectable()
export class FileSystemDashboard extends BaseDashboard {
    @inject(FileSystem) protected readonly fileSystem: FileSystem;
    @inject(FileSystemDashboardQueue) protected readonly queue: FileSystemDashboardQueue;

    protected async resolveDashboardItems(): Promise<DashboardItem[]> {
        const items: DashboardItem[] = [];
        const home = await this.fileSystem.getCurrentUserHome();
        const directory = await this.fileSystem.getFileStat(home!.uri);

        for (const child of directory!.children!) {
            items.push(this.toDashboardItem(child));
        }

        return items;
    }

    protected toDashboardItem(stat: FileStat): DashboardItem {
        return {
            uri: stat.uri,
            name: new URI(FileUri.fsPath(stat.uri)).displayName,
            visible: !stat.uri.startsWith('.'),
            iconClass: this.queue.isQueued(stat.uri) ? PROCESS_ITEM_CLASS : DEFAULT_ITEM_CLASS
        };
    }
}
