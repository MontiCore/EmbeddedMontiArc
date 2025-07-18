/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { PathsServer } from "../common/paths-protocol";
import { PathsRegistry } from "./paths-registry";

@injectable()
export class PathsServerImpl implements PathsServer {
    @inject(PathsRegistry) protected readonly registry: PathsRegistry;

    public async getPath(id: string): Promise<string> {
        return this.registry.getPath(id).toString(true);
    }
}
