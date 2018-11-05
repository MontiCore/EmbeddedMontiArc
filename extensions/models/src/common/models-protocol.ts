/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { FileStat } from "@theia/filesystem/lib/common";

export const MODELS_PATH = "/services/emastudio-models";

export const ModelsServer = Symbol("ModelsServer");

export interface ModelsServer {
    getModels(): Promise<FileStat[]>;
}
