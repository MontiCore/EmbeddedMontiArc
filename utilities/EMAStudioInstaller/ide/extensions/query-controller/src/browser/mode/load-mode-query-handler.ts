/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable } from "inversify";
import { ModeQueryHandler } from "./mode-controller";
import URI from "@elysium/core/lib/common/uri";
import { FrontendApplicationStateService } from "@theia/core/lib/browser/frontend-application-state";
import { FileSystem } from "@theia/filesystem/lib/common";
import { ZIPDownloader, DownloadConfig } from "@elysium/downloader/lib/browser";
import { ExtendedWindowService } from "@elysium/core/lib/browser";
import { FileSystemDashboardWidget } from "@elysium/dashboard/lib/browser/filesystem";
import { Widget } from "@theia/core/lib/browser";
import { FileUri } from "@theia/core/lib/node/file-uri";

@injectable()
export class LoadModeQueryHandler implements ModeQueryHandler {
    @inject(FrontendApplicationStateService) protected readonly stateService: FrontendApplicationStateService;
    @inject(FileSystem) protected readonly fileSystem: FileSystem;
    @inject(ZIPDownloader) protected readonly downloader: ZIPDownloader;
    @inject(ExtendedWindowService) protected readonly windowService: ExtendedWindowService;
    @inject(FileSystemDashboardWidget) protected readonly widget: FileSystemDashboardWidget;

    public canHandle(uri: URI): boolean {
        return uri.getQueryParam("mode") === "load";
    }

    public async handle(uri: URI): Promise<void> {
        const root = uri.getQueryParam("root");
        const url = uri.getQueryParam("url");

        if (root) return this.handleWorkspace(root, url);
        else console.warn("[Load Mode]: The specification of a root is mandatory.");
    }

    protected async handleWorkspace(root: string, url?: string): Promise<void> {
        await this.stateService.reachedState("initialized_layout");

        if (await this.fileSystem.exists(root)) {
            Widget.attach(this.widget, document.body);
            this.handleRedirect();
        } else {
            if (url) return this.handleOnlineWorkspace(root, url);
            else return this.handleLocalWorkspace(root);
        }
    }

    protected async handleLocalWorkspace(root: string): Promise<void> {
        const rootURI = FileUri.create(root);
        const rootDisplayName = rootURI.displayName;

        Widget.attach(this.widget, document.body);

        await this.widget.model.create(rootDisplayName);

        this.handleRedirect();
    }

    protected async handleOnlineWorkspace(root: string, url: string): Promise<void> {
        const config: DownloadConfig = { localURI: root, remoteURI: url };
        const rootURI = FileUri.create(root);
        const rootDisplayName = rootURI.displayName;
        const rootWithScheme = rootURI.toString(true);

        Widget.attach(this.widget, document.body);

        this.widget.queue.addToQueue(rootWithScheme);
        await this.widget.model.create(rootDisplayName);
        await this.downloader.download(config);
        this.widget.queue.removeFromQueue(rootWithScheme);

        this.handleRedirect();
    }

    protected handleRedirect(): void {
        const locationURI = new URI(window.location.href);
        const params = locationURI.getQueryParams();

        params.delete("mode");
        params.delete("url");

        const targetURI = locationURI.setQueryParams(params).toString(true);

        this.windowService.redirectWindow(targetURI);
    }
}
