/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { FileSystemDashboard } from "@elysium/dashboard/lib/browser";
import { DashboardItem } from "@elysium/dashboard/lib/browser";
import { ModelsServer } from "@emastudio/models/lib/common";
import { FileStat } from "@theia/filesystem/lib/common";
import { FileUri } from "@theia/core/lib/node/file-uri";
import { DEFAULT_ITEM_CLASS } from "@elysium/dashboard/lib/browser";

@injectable()
export class EMAStudioDashboard extends FileSystemDashboard {
    @inject(ModelsServer) protected readonly server: ModelsServer;

    protected async resolveDashboardItems(): Promise<DashboardItem[]> {
        const items: DashboardItem[] = [];
        const stats = await this.server.getModels();

        for (const stat of stats) {
            items.push(this.toDashboardItem(stat));
        }

        return items;
    }

    protected toDashboardItem(stat: FileStat): DashboardItem {
        return {
            uri: stat.uri,
            name: FileUri.create(stat.uri).displayName,
            visible: !stat.uri.startsWith('.'),
            iconClass: DEFAULT_ITEM_CLASS
        };
    }
}
