/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import {
    FileSystemWatcherServer,
    WatchOptions,
    FileSystemWatcherClient,
    FileChange,
    FileChangeType
} from "@theia/filesystem/lib/common/filesystem-watcher-protocol";
import URI from "@theia/core/lib/common/uri";

@injectable()
export class BrowserFileSystemWatcher implements FileSystemWatcherServer {
    protected watcherId: number = 0;
    protected clients: FileSystemWatcherClient[] = [];
    protected changes: FileChange[] = [];
    protected timer: number;

    public async watchFileChanges(uri: string, options?: WatchOptions): Promise<number> {
        // TODO: Map WatchOptions.
        return this.watcherId++;
    }

    public async unwatchFileChanges(watcher: number): Promise<void> {
        return undefined;
    }

    public setClient(client: FileSystemWatcherClient | undefined): void {
        if (client) this.clients.push(client);
    }

    public dispose(): void {
        // NOOP
    }

    public trigger(): void {
        for (const client of this.clients) {
            if (client) client.onDidFilesChanged({ changes: this.changes });
        }

        this.changes = [];
    }

    protected scheduleTrigger(): void {
        window.clearTimeout(this.timer);

        this.timer = window.setTimeout(this.trigger.bind(this), 200);
    }

    public addModified(uri: string): void {
        this.scheduleChanges(uri, FileChangeType.UPDATED, false);
    }

    public addCreated(uri: string): void {
        this.scheduleChanges(uri, FileChangeType.ADDED);
    }

    public addDeleted(uri: string): void {
        this.scheduleChanges(uri, FileChangeType.DELETED);
    }

    protected scheduleChanges(uri: string, type: FileChangeType, parent: boolean = true): void {
        // TODO: Check WatchOptions.
        const urio = new URI(uri);
        const change = { uri, type };

        this.addChange(change);

        if (parent) {
            const parentURI = urio.parent.toString();
            const parentChange = {uri: parentURI, type: FileChangeType.UPDATED};

            this.addChange(parentChange);
        }

        this.scheduleTrigger();
    }

    protected addChange(change: FileChange): void {
        if (change.type === FileChangeType.DELETED) {
            this.changes = this.changes.filter(value => value.uri !== change.uri);
        }

        this.changes.push(change);
    }
}
