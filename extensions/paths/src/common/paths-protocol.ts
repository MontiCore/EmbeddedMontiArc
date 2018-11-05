/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

export const PATHS_PATH = "/services/emastudio-paths";

export const PathsServer = Symbol("PathsServer");

export interface PathsServer {
    getPath(id: string): Promise<string>;
}
