/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { ScriptsConditions } from "@emastudio/scripts/lib/browser";
import { Disposable } from "@theia/core/lib/common";
import { VISUALIZE_SCRIPT } from "../common";

@injectable()
export class VisualizationConditions {
    @inject(ScriptsConditions) protected conditions: ScriptsConditions;

    public async check(disposable: Disposable): Promise<void> {
        const existsScript = await this.conditions.check("visualization", VISUALIZE_SCRIPT);

        if (!existsScript) disposable.dispose();
    }
}
