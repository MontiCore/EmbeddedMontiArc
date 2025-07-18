/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { PathsRegistry, PathContribution } from "@emastudio/paths/lib/node";
import { InteractiveSimulatorPaths } from "../common";

@injectable()
export class InteractiveSimulatorPathContribution implements PathContribution {
    public registerTo(registry: PathsRegistry): void {
        registry.setPath(InteractiveSimulatorPaths.INTERACTIVESIMULATOR, registry.getRoot().resolve("interactiveSimulator"));
    }
}
