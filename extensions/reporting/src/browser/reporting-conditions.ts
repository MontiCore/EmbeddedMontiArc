/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { ScriptsConditions } from "@emastudio/scripts/lib/browser";
import { Disposable } from "@theia/core/lib/common";
import { REPORT_SCRIPT, REPORT_STREAMS_SCRIPT } from "../common";

@injectable()
export class ReportingConditions {
    @inject(ScriptsConditions) protected readonly conditions: ScriptsConditions;

    public async check(disposable: Disposable): Promise<void> {
        const existsScript = await this.conditions.check("reporting", REPORT_SCRIPT);

        if (!existsScript) disposable.dispose();
    }

    public async checkStreams(disposable: Disposable): Promise<void> {
        const existsScript = await this.conditions.check("reporting", REPORT_STREAMS_SCRIPT);

        if (!existsScript) disposable.dispose();
    }
}
