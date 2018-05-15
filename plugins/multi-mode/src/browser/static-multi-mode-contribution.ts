/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { BaseMultiModeContribution } from "./multi-mode";
import URI from "@elysium/core/lib/common/uri";
import { FileSystem } from "@theia/filesystem/lib/common";
import { FrontendApplicationStateService } from "@theia/core/lib/browser/frontend-application-state";

@injectable()
export class StaticMultiModeContribution extends BaseMultiModeContribution {
    @inject(FileSystem) protected readonly fileSystem: FileSystem;
    @inject(FrontendApplicationStateService) protected readonly stateService: FrontendApplicationStateService;

    public readonly mode: string = "static";

    public async handle(uri: URI): Promise<void> {
        const root = uri.getQueryParam("root");
        const path = uri.getQueryParam("path");

        if (root && path) return this.handlePaths(root, path);
        else console.warn("[Static Mode]: A root and a path are required as query params in the url.");
    }

    protected async handlePaths(root: string, path: string): Promise<void> {
        const fullPath = new URI(root).resolve(path).toString();

        if (await this.fileSystem.exists(fullPath)) return this.handleFullPath(fullPath);
        else console.warn("[Static Mode]: The file does not exist.");
    }

    protected async handleFullPath(fullPath: string): Promise<void> {
        const resolvings = await this.fileSystem.resolveContent(fullPath);
        const doc = document.open("text/html", "replace");

        await this.stateService.reachedState("initialized_layout");

        doc.write(resolvings.content);
        doc.close();
    }
}
