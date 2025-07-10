/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { PathContribution, PathsRegistry } from "@emastudio/paths/lib/node";

export const MODELS_PATH_ID: string = "models";

@injectable()
export class ModelsPathContribution implements PathContribution {
    public registerTo(registry: PathsRegistry): void {
        registry.setPath(MODELS_PATH_ID, registry.getRoot().resolve("models"));
    }
}
