/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import {
    FileSystemWatcherServerProxy,
    FileSystemWatcherServer
} from "@theia/filesystem/lib/common/filesystem-watcher-protocol";
import { FileSystem } from "@theia/filesystem/lib/common/filesystem";
import { BrowserFileSystemWatcher } from "./browser-filesystem-watcher";
import { FileSystemBrowser } from "./browser-filesystem";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    unbind(FileSystemWatcherServerProxy);

    bind(BrowserFileSystemWatcher).toSelf().inSingletonScope();
    bind(FileSystemBrowser).toSelf().inSingletonScope();

    rebind(FileSystemWatcherServer).to(BrowserFileSystemWatcher).inSingletonScope();
    rebind(FileSystem).toDynamicValue(
        ctx => ctx.container.get(FileSystemBrowser)
    ).inSingletonScope();
});
