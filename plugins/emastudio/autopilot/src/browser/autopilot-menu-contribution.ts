/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MenuContribution, MenuModelRegistry } from "@theia/core/lib/common/menu";
import { CommonMenus } from "@emastudio/core/lib/browser";
import { AutoPilotCommands } from "./autopilot-command-contribution";
import { AutoPilotConditions } from "./autopilot-conditions";

export namespace AutoPilotMenus {
    export const EXECUTE_DISTRIBUTED = [...CommonMenus.FEATURES, "1_executing"];
}

@injectable()
export class AutoPilotMenuContribution implements MenuContribution {
    @inject(AutoPilotConditions) protected readonly conditions: AutoPilotConditions;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(AutoPilotMenus.EXECUTE_DISTRIBUTED, {
            commandId: AutoPilotCommands.EXECUTE_DISTRIBUTED.id
        });

        return this.conditions.check(menu);
    }
}
