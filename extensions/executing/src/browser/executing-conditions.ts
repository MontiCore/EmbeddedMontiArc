/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { ScriptsConditions } from "@emastudio/scripts/lib/browser";
import { Disposable } from "@theia/core/lib/common";
import { EXECUTE_SCRIPT } from "../common";

@injectable()
export class ExecutingConditions {
    @inject(ScriptsConditions) protected readonly conditions: ScriptsConditions;

    public async check(disposable: Disposable): Promise<void> {
        const existsScript = await this.conditions.check("executing", EXECUTE_SCRIPT);

        if (!existsScript) disposable.dispose();
    }
}
