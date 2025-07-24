/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { StaticContribution } from "@emastudio/static/lib/node/static-registry";
import URI from "@theia/core/lib/common/uri";
import { PathsRegistry } from "@emastudio/paths/lib/node";
import { SUPERMARIO_STATIC_PATH, SUPERMARIO_PATH_ID } from "../common";

@injectable()
export class SuperMarioStaticContribution implements StaticContribution {
    public readonly path: string;
    public readonly uri: URI;

    public constructor(
        @inject(PathsRegistry) registry: PathsRegistry
    ) {
        this.path = SUPERMARIO_STATIC_PATH;
        this.uri = registry.getPath(SUPERMARIO_PATH_ID);
    }
}
