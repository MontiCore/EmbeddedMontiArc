/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable, postConstruct } from "inversify";
import { BaseDashboardModel } from "../dashboard-model";
import { DashboardItem } from "../dashboard";
import { Event, Emitter } from "@theia/core/lib/common";
import { DemosDownloader, DemosDownloadConfig } from "@elysium/downloader/lib/browser/demos-downloader";
import { FileSystem } from "@theia/filesystem/lib/common";

@injectable()
export class DemosDashboardModel extends BaseDashboardModel {
    @inject(FileSystem) protected readonly fileSystem: FileSystem;
    @inject(DemosDownloader) protected readonly downloader: DemosDownloader;

    protected readonly onCloneEmitter: Emitter<string> = new Emitter<string>();
    protected readonly onClonedEmitter: Emitter<string> = new Emitter<string>();

    @postConstruct()
    protected init(): void {
        super.init();

        this.toDispose.push(this.onCloneEmitter);
        this.toDispose.push(this.onClonedEmitter);
    }

    public async clone(uri: string, item: DashboardItem): Promise<void> {
        const config: DemosDownloadConfig = { localURI: uri, demo: item.name };

        await this.fileSystem.createFolder(uri);

        this.fireClone(uri);
        await this.downloader.download(config);
        this.fireCloned(uri);
    }

    public get onClone(): Event<string> {
        return this.onCloneEmitter.event;
    }

    protected fireClone(target: string): void {
        this.onCloneEmitter.fire(target);
    }

    public get onCloned(): Event<string> {
        return this.onClonedEmitter.event;
    }

    protected fireCloned(target: string): void {
        this.onClonedEmitter.fire(target);
    }
}
