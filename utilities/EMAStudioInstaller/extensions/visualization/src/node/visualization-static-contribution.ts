/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { StaticContribution } from "@emastudio/static/lib/node/static-registry";
import { PathsRegistry } from "@emastudio/paths/lib/node";
import { VISUALIZATION_STATIC_PATH, VISUALIZATION_PATH_ID } from "../common";

import URI from "@theia/core/lib/common/uri";

@injectable()
export class VisualizationStaticContribution implements StaticContribution {
    public readonly path: string;
    public readonly uri: URI;

    public constructor(
        @inject(PathsRegistry) registry: PathsRegistry
    ) {
        this.path = VISUALIZATION_STATIC_PATH;
        this.uri = registry.getPath(VISUALIZATION_PATH_ID).resolve("SVG");
    }
}
