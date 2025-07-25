/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseLanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { MONTIMATH_LANGUAGE_ID, MONTIMATH_LANGUAGE_NAME } from "../common";

@injectable()
export class MontiMathLanguageWorkerContribution extends BaseLanguageWorkerContribution {
    public readonly id: string = MONTIMATH_LANGUAGE_ID;
    public readonly name: string = MONTIMATH_LANGUAGE_NAME;
}
