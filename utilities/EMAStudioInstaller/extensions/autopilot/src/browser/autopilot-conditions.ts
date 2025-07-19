/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { ScriptsConditions } from "@emastudio/scripts/lib/browser";
import { Disposable } from "@theia/core/lib/common";
import { EXECUTE_DISTRIBUTED_SCRIPT, EXECUTE_MODELICA_SCRIPT } from "../common";

@injectable()
export class AutoPilotConditions {
    @inject(ScriptsConditions) protected readonly conditions: ScriptsConditions;

    public async checkDistributed(disposable: Disposable): Promise<void> {
        const existsScript = await this.conditions.check("executing", EXECUTE_DISTRIBUTED_SCRIPT);

        if (!existsScript) disposable.dispose();
    }

    public async checkModelica(disposable: Disposable): Promise<void> {
        const existsScript = await this.conditions.check("executing", EXECUTE_MODELICA_SCRIPT);

        if (!existsScript) disposable.dispose();
    }
}
