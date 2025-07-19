/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { WorkspaceServer } from "@theia/workspace/lib/common";
import { injectable, inject, postConstruct } from "inversify";
import URI from "@elysium/core/lib/common/uri";
import { FileUri } from "@theia/core/lib/node/file-uri";
import { FileSystemDashboardWidget } from "@elysium/dashboard/lib/browser";
import { Widget } from "@theia/core/lib/browser";

@injectable()
export class BrowserWorkspace implements WorkspaceServer {
    @inject(FileSystemDashboardWidget) protected readonly widget: FileSystemDashboardWidget;

    protected root: Promise<string | undefined>;

    @postConstruct()
    protected init(): void {
        const uri = new URI(window.location.href);
        const root = uri.getQueryParam("root");
        const rootUri = root ? FileUri.create(root).toString() : undefined;

        this.root = Promise.resolve(rootUri);

        if (!rootUri && !this.widget.isAttached) Widget.attach(this.widget, document.body);
    }

    public async getMostRecentlyUsedWorkspace(): Promise<string | undefined> {
        return this.root;
    }

    public async setMostRecentlyUsedWorkspace(uri: string): Promise<void> {
        this.root = Promise.resolve(uri);
    }

    public async getRecentWorkspaces(): Promise<string[]> {
        const root = await this.root;

        return root ? [root] : [];
    }
}
