/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable } from "inversify";
import { FrontendApplication, FrontendApplicationContribution } from "@theia/core/lib/browser";
import { FileSystem } from "@theia/filesystem/lib/common";
import { NotificationsMessageClient } from "@theia/messages/lib/browser/notifications-message-client";
import { MessageType } from "@theia/core/lib/common";

import IndexedDBFileSystem from "browserfs/dist/node/backend/IndexedDB";

@injectable()
export class BrowserFilesystemFrontendContribution implements FrontendApplicationContribution {
    @inject(FileSystem) protected readonly fileSystem: FileSystem;
    @inject(NotificationsMessageClient) protected readonly notifications: NotificationsMessageClient;

    public onStart(application: FrontendApplication): void {
        this.checkAvailability();
    }

    protected checkAvailability(): void {
        const isFileSystemAvailable = IndexedDBFileSystem.isAvailable();

        if (isFileSystemAvailable) this.checkFunctionality();
        else this.showNotification();
    }

    protected checkFunctionality(): void {
        this.fileSystem.exists("file:///").catch(this.showNotification);
    }

    protected showNotification(): void {
        this.notifications.showMessage({
            type: MessageType.Error,
            text: `The virtual filesystem is not available on your system. A browser's private mode or
                       settings which prohibit placement of cookies could be at fault here.`,
            options: {
                timeout: 60 * 60 * 1000
            }
        }).catch(error => console.error(error.message));
    }
}
