/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseMultiModeContribution } from "./multi-mode";
import URI from "@elysium/core/lib/common/uri";

@injectable()
export class LoadMultiModeContribution extends BaseMultiModeContribution {
    public readonly mode: string = "load";

    public async handle(uri: URI): Promise<void> {
    }
}
