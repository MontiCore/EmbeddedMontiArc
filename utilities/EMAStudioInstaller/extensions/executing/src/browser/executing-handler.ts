/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { EXECUTE_SCRIPT } from "../common";
import { ScriptsService } from "@emastudio/scripts/lib/browser";

export const ExecutingHandler = Symbol("ExecutingHandler");
export interface ExecutingHandler {
    isEnabled(): Promise<boolean>;
    execute(): Promise<void>;
}

@injectable()
export class DefaultExecutingHandler implements ExecutingHandler {
    @inject(ScriptsService) protected readonly scriptsService: ScriptsService;

    public async isEnabled(): Promise<boolean> {
        return true;
    }

    public async execute(): Promise<void> {
        await this.scriptsService.execute({
            label: "Execution",
            plugin: "executing",
            script: EXECUTE_SCRIPT
        });
    }
}
