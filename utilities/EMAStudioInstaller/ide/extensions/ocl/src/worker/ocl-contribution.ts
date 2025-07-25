/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseLanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { OCL_LANGUAGE_ID, OCL_LANGUAGE_NAME } from "../common";

@injectable()
export class OCLLanguageWorkerContribution extends BaseLanguageWorkerContribution {
    public readonly id: string = OCL_LANGUAGE_ID;
    public readonly name: string = OCL_LANGUAGE_NAME;
}
