/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { LanguageWorkerContribution } from "@elysium/languages/lib/worker";
import { EMBEDDEDMONTIARCDL_LANGUAGE_ID, EMBEDDEDMONTIARCDL_LANGUAGE_NAME } from "../common";
import { AnalyzeItem, OutlineItem } from "@elysium/languages/lib/common";

@injectable()
export class EmbeddedMontiArcDLLanguageWorkerContribution implements LanguageWorkerContribution {
    public readonly id: string = EMBEDDEDMONTIARCDL_LANGUAGE_ID;
    public readonly name: string = EMBEDDEDMONTIARCDL_LANGUAGE_NAME;
    public readonly isReady: Promise<boolean> = Promise.resolve(true);

    public parse(docValue: string): void {
        // NOOP
    }

    public analyze(): AnalyzeItem[] {
        return [];
    }

    public outline(): OutlineItem[] {
        return [];
    }
}
