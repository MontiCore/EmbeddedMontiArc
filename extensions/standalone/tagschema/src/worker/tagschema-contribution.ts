/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseLanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { TAGSCHEMA_LANGUAGE_ID, TAGSCHEMA_LANGUAGE_NAME } from "../common";

@injectable()
export class TagSchemaLanguageWorkerContribution extends BaseLanguageWorkerContribution {
    public readonly id: string = TAGSCHEMA_LANGUAGE_ID;
    public readonly name: string = TAGSCHEMA_LANGUAGE_NAME;
}
