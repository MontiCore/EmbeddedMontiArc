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
import { BrowserFilesystemFrontendContribution } from "./browser-filesystem-frontend-contribution";
import { FrontendApplicationContribution } from "@theia/core/lib/browser";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    unbind(FileSystemWatcherServerProxy);

    bind(BrowserFileSystemWatcher).toSelf().inSingletonScope();
    bind(FileSystemBrowser).toSelf().inSingletonScope();

    rebind(FileSystemWatcherServer).to(BrowserFileSystemWatcher).inSingletonScope();
    rebind(FileSystem).toDynamicValue(
        ctx => ctx.container.get(FileSystemBrowser)
    ).inSingletonScope();

    bind(BrowserFilesystemFrontendContribution).toSelf().inSingletonScope();
    bind(FrontendApplicationContribution).toDynamicValue(
        ctx => ctx.container.get(BrowserFilesystemFrontendContribution)
    ).inSingletonScope();
});
