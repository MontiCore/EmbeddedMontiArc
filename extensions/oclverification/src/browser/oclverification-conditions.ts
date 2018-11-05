/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { ScriptsConditions } from "@emastudio/scripts/lib/browser";
import { Disposable } from "@theia/core/lib/common";
import { CHECK_SCRIPT, VISUALIZE_SCRIPT } from "../common";

@injectable()
export class OCLVerificationConditions {
    @inject(ScriptsConditions) protected readonly conditions: ScriptsConditions;

    public async checkCheck(disposable: Disposable): Promise<void> {
        const existsScript = await this.conditions.check("oclverification", CHECK_SCRIPT);

        if (!existsScript) disposable.dispose();
    }

    public async checkVisualize(disposable: Disposable): Promise<void> {
        const existsScript = await this.conditions.check("oclverification", VISUALIZE_SCRIPT);

        if (!existsScript) disposable.dispose();
    }
}
