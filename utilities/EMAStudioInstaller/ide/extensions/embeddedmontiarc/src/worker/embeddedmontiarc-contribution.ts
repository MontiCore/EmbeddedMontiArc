/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseLanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { EMBEDDEDMONTIARC_LANGUAGE_ID, EMBEDDEDMONTIARC_LANGUAGE_NAME } from "../common";

@injectable()
export class EmbeddedMontiArcLanguageWorkerContribution extends BaseLanguageWorkerContribution {
    public readonly id: string = EMBEDDEDMONTIARC_LANGUAGE_ID;
    public readonly name: string = EMBEDDEDMONTIARC_LANGUAGE_NAME;
}
