/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseLanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { CD_LANGUAGE_ID, CD_LANGUAGE_NAME } from "../common";

@injectable()
export class CDLanguageWorkerContribution extends BaseLanguageWorkerContribution {
    public readonly id: string = CD_LANGUAGE_ID;
    public readonly name: string = CD_LANGUAGE_NAME;
}
