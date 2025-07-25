/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { inject, injectable } from "inversify";
import { ModelsServer } from "../common";
import { FileStat, FileSystem } from "@theia/filesystem/lib/common";
import { PathsRegistry } from "@emastudio/paths/lib/node";
import { MODELS_PATH_ID } from "./models-path-contribution";

@injectable()
export class ModelsServerImpl implements ModelsServer {
    @inject(PathsRegistry) protected readonly registry: PathsRegistry;
    @inject(FileSystem) protected readonly fileSystem: FileSystem;

    public async getModels(): Promise<FileStat[]> {
        const modelsURI = this.registry.getPath(MODELS_PATH_ID).toString();
        const rootDirectory = await this.fileSystem.getFileStat(modelsURI);

        return rootDirectory ? rootDirectory.children! : [];
    }
}
