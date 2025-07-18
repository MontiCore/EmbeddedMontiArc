/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { StaticContribution } from "@emastudio/static/lib/node/static-registry";
import { PathsRegistry } from "@emastudio/paths/lib/node";
import { InteractiveSimulatorPaths, InteractiveSimulatorStaticPaths } from "../common";

import URI from "@theia/core/lib/common/uri";

@injectable()
export class InteractiveSimulatorStaticContribution implements StaticContribution {
    public readonly path: string;
    public readonly uri: URI;

    public constructor(
        @inject(PathsRegistry) registry: PathsRegistry
    ) {
        this.path = InteractiveSimulatorStaticPaths.INTERACTIVESIMULATOR;
        this.uri = registry.getPath(InteractiveSimulatorPaths.INTERACTIVESIMULATOR);
    }
}
