/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { PathContribution } from "@emastudio/paths/lib/node/paths-registry";
import { PathsRegistry } from "@emastudio/paths/lib/node";
import { TESTING_PATH_ID } from "../common";

@injectable()
export class TestingPathContribution implements PathContribution {
    public registerTo(registry: PathsRegistry): void {
        registry.setPath(TESTING_PATH_ID, registry.getRoot().resolve("testResults"));
    }
}
