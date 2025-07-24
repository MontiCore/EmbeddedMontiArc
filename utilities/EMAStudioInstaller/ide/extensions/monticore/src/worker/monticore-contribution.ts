/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseLanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { MONTICORE_LANGUAGE_ID, MONTICORE_LANGUAGE_NAME } from "../common";

@injectable()
export class MontiCoreLanguageWorkerContribution extends BaseLanguageWorkerContribution {
    public readonly id: string = MONTICORE_LANGUAGE_ID;
    public readonly name: string = MONTICORE_LANGUAGE_NAME;
}
