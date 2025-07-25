/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { StaticContribution } from "@emastudio/static/lib/node/static-registry";
import { PathsRegistry } from "@emastudio/paths/lib/node";
import { OCLVERIFICATION_STATIC_PATH, OCLVERIFICATION_PATH_ID } from "../common";

import URI from "@theia/core/lib/common/uri";

@injectable()
export class OCLVerificationStaticContribution implements StaticContribution {
    public readonly path: string;
    public readonly uri: URI;

    public constructor(
        @inject(PathsRegistry) registry: PathsRegistry
    ) {
        this.path = OCLVERIFICATION_STATIC_PATH;
        this.uri = registry.getPath(OCLVERIFICATION_PATH_ID);
    }
}
