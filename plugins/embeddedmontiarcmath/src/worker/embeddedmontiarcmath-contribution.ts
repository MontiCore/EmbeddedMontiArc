/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseLanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { EMBEDDEDMONTIARCMATH_LANGUAGE_ID, EMBEDDEDMONTIARCMATH_LANGUAGE_NAME } from "../common";

@injectable()
export class EmbeddedMontiArcMathLanguageWorkerContribution extends BaseLanguageWorkerContribution {
    public readonly id: string = EMBEDDEDMONTIARCMATH_LANGUAGE_ID;
    public readonly name: string = EMBEDDEDMONTIARCMATH_LANGUAGE_NAME;
}
