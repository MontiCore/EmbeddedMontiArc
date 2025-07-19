/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseLanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { TAGGING_LANGUAGE_ID, TAGGING_LANGUAGE_NAME } from "../common";

@injectable()
export class TaggingLanguageWorkerContribution extends BaseLanguageWorkerContribution {
    public readonly id: string = TAGGING_LANGUAGE_ID;
    public readonly name: string = TAGGING_LANGUAGE_NAME;
}
