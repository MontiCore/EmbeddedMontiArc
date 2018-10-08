/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { PathContribution, PathsRegistry } from "./paths-registry";
import { FileUri } from "@theia/core/lib/node/file-uri";

import * as Path from "path";
import * as FileSystem from "fs-extra";

@injectable()
export class RootPathContribution implements PathContribution {
    public registerTo(registry: PathsRegistry): void {
        const resourcesPath = Path.join(process.cwd(), "resources");
        const debugRoot = process.env.EMASTUDIO_ROOT;

        let rootURI = undefined;

        if (FileSystem.pathExistsSync(resourcesPath)) rootURI = FileUri.create(resourcesPath);
        else if (debugRoot) rootURI = FileUri.create(debugRoot);
        else rootURI = FileUri.create(resourcesPath);

        registry.setPath('.', rootURI);
    }
}
