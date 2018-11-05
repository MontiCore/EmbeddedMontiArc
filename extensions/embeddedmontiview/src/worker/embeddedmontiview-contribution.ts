/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseLanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { EMBEDDEDMONTIVIEW_LANGUAGE_ID, EMBEDDEDMONTIVIEW_LANGUAGE_NAME } from "../common";

@injectable()
export class EmbeddedMontiViewLanguageWorkerContribution extends BaseLanguageWorkerContribution {
    public readonly id: string = EMBEDDEDMONTIVIEW_LANGUAGE_ID;
    public readonly name: string = EMBEDDEDMONTIVIEW_LANGUAGE_NAME;
}
