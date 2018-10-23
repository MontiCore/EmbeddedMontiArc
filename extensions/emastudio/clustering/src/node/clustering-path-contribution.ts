/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { PathContribution } from "@emastudio/paths/lib/node/paths-registry";
import { PathsRegistry } from "@emastudio/paths/lib/node";
import { CLUSTER_FIDDLE_PATH_ID, CLUSTERING_EXEC_PATH_ID } from "../common";

@injectable()
export class ClusteringPathContribution implements PathContribution {
    public registerTo(registry: PathsRegistry): void {
        registry.setPath(CLUSTER_FIDDLE_PATH_ID, registry.getRoot().resolve("cluster-fiddle"));
        registry.setPath(CLUSTERING_EXEC_PATH_ID, registry.getRoot().resolve("exec"));
    }
}
