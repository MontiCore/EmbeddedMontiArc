/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { PathContribution } from "@emastudio/paths/lib/node/paths-registry";
import { PathsRegistry } from "@emastudio/paths/lib/node";
import { SCRIPTS_PATH_ID } from "../common/scripts-protocol";

@injectable()
export class ScriptsPathContribution implements PathContribution {
    public registerTo(registry: PathsRegistry): void {
        registry.setPath(SCRIPTS_PATH_ID, registry.getRoot().resolve("scripts"));
    }
}
