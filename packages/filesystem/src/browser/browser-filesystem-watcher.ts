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
        // TODO: Check WatchOptions.
        const change = { uri, type: FileChangeType.UPDATED };

        this.addChange(change);
        this.scheduleTrigger();
    }

    public addCreated(uri: string): void {
        // TODO: Check WatchOptions.
        const urio = new URI(uri);
        const parentURI = urio.parent.toString();
        const change = { uri, type: FileChangeType.ADDED };
        const parentChange = { uri: parentURI, type: FileChangeType.UPDATED };

        this.addChanges(change, parentChange);
        this.scheduleTrigger();
    }

    public addDeleted(uri: string): void {
        // TODO: Check WatchOptions.
        const urio = new URI(uri);
        const parentURI = urio.parent.toString();
        const change = { uri, type: FileChangeType.DELETED };
        const parentChange = { uri: parentURI, type: FileChangeType.UPDATED };

        this.addChanges(change, parentChange);
        this.scheduleTrigger();
    }

    protected addChanges(...changes: FileChange[]): void {
        for (const change of changes) {
            this.addChange(change);
        }
    }

    protected addChange(change: FileChange): void {
        if (change.type === FileChangeType.DELETED) {
            this.changes = this.changes.filter(value => value.uri !== change.uri);
        }

        this.changes.push(change);
    }
}
