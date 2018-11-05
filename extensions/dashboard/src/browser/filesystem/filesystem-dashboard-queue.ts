/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";

@injectable()
export class FileSystemDashboardQueue {
    protected queue: Set<string> = new Set<string>();

    public isBusy(): boolean {
        return this.queue.size > 0;
    }

    public isQueued(uri: string): boolean {
        return this.queue.has(uri);
    }

    public addToQueue(uri: string): void {
        this.queue.add(uri);
    }

    public removeFromQueue(uri: string): void {
        this.queue.delete(uri);
    }
}
