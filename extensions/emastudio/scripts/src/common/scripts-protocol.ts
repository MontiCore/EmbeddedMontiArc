/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

export const SCRIPTS_PATH_ID: string = "scripts";

import { JsonRpcServer } from "@theia/core/lib/common/messaging";

export const SCRIPTS_PATH = "/services/emastudio-scripts";

export const ScriptsClient = Symbol("ScriptsClient");

export interface ScriptsClient {
}

export const ScriptsServer = Symbol("ScriptsServer");

export interface ScriptsServer extends JsonRpcServer<ScriptsClient> {
}
