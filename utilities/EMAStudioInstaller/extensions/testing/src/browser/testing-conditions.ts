/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { ScriptsConditions } from "@emastudio/scripts/lib/browser";
import { Disposable } from "@theia/core/lib/common";
import { TESTING_SCRIPT, TESTING_ALL_SCRIPT } from "../common";

@injectable()
export class TestingConditions {
    @inject(ScriptsConditions) protected readonly conditions: ScriptsConditions;

    public async check(disposable: Disposable): Promise<void> {
        const existsScript = await this.conditions.check("testing", TESTING_SCRIPT);

        if (!existsScript) disposable.dispose();
    }

    public async checkAll(disposable: Disposable): Promise<void> {
        const existsScript = await this.conditions.check("testing", TESTING_ALL_SCRIPT);

        if (!existsScript) disposable.dispose();
    }
}
