/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { BaseLanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { MACOCOVIZ_LANGUAGE_ID, MACOCOVIZ_LANGUAGE_NAME } from "../common";

@injectable()
export class MaCoCoVIZLanguageWorkerContribution extends BaseLanguageWorkerContribution {
    public readonly id: string = MACOCOVIZ_LANGUAGE_ID;
    public readonly name: string = MACOCOVIZ_LANGUAGE_NAME;
}
