/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseLanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { STREAMUNITS_LANGUAGE_ID, STREAMUNITS_LANGUAGE_NAME } from "../common";

@injectable()
export class StreamUnitsLanguageWorkerContribution extends BaseLanguageWorkerContribution {
    public readonly id: string = STREAMUNITS_LANGUAGE_ID;
    public readonly name: string = STREAMUNITS_LANGUAGE_NAME;
}
