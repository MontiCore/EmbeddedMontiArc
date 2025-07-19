/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { ScriptsConditions } from "@emastudio/scripts/lib/browser";
import { Disposable } from "@theia/core/lib/common";
import { InteractiveSimulatorScripts } from "../common";

@injectable()
export class InteractiveSimulatorConditions {
    @inject(ScriptsConditions) protected readonly conditions: ScriptsConditions;

    public async checkDebug(disposable: Disposable): Promise<void> {
        const existsScript = await this.conditions.check("interactivesimulator", InteractiveSimulatorScripts.DEBUG);

        if (!existsScript) disposable.dispose();
    }

    public async checkDebugWosvg(disposable: Disposable): Promise<void> {
        const existsScript = await this.conditions.check("interactivesimulator", InteractiveSimulatorScripts.DEBUG_WOSVG);

        if (!existsScript) disposable.dispose();
    }
}
