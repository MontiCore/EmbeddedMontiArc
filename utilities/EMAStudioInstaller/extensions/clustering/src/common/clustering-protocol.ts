/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { isWindows } from "@theia/core/lib/common";

export const CLUSTERING_PATH: string = "/services/emastudio-clustering";

export const CLUSTER_FIDDLE_PATH_ID: string = "cluster-fiddle";
export const CLUSTERING_EXEC_PATH_ID: string = "exec";

export const CLUSTERING_STATIC_PATH: string = "/clustering";

export const RUN_SCRIPT: string = "run" + (isWindows ? ".bat" : ".sh");
