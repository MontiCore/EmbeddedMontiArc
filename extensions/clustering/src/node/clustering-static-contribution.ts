/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { StaticContribution } from "@emastudio/static/lib/node";
import { PathsRegistry } from "@emastudio/paths/lib/node";
import { CLUSTER_FIDDLE_PATH_ID, CLUSTERING_STATIC_PATH } from "../common";

import URI from "@theia/core/lib/common/uri";

@injectable()
export class ClusteringStaticContribution implements StaticContribution {
    public readonly path: string;
    public readonly uri: URI;

    public constructor(
        @inject(PathsRegistry) registry: PathsRegistry
    ) {
        this.path = CLUSTERING_STATIC_PATH;
        this.uri = registry.getPath(CLUSTER_FIDDLE_PATH_ID);
    }
}
