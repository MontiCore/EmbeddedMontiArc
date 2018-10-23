/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MenuContribution, MenuModelRegistry } from "@theia/core/lib/common/menu";
import { CommonMenus } from "@emastudio/core/lib/browser";
import { AutoPilotConditions } from "./autopilot-conditions";
import { AutoPilotCommands } from "./autopilot-command-contribution";

export namespace AutoPilotMenus {
    export const EXECUTE = [...CommonMenus.FEATURES, "1_executing"];
}

@injectable()
export class AutoPilotMenuContribution implements MenuContribution {
    @inject(AutoPilotConditions) protected readonly conditions: AutoPilotConditions;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        await Promise.all([
            this.registerMenuDistributed(registry),
            this.registerMenuModelica(registry)
        ]);
    }

    protected async registerMenuDistributed(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(AutoPilotMenus.EXECUTE, {
            commandId: AutoPilotCommands.EXECUTE_DISTRIBUTED.id
        });

        return this.conditions.checkDistributed(menu);
    }

    protected async registerMenuModelica(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(AutoPilotMenus.EXECUTE, {
            commandId: AutoPilotCommands.EXECUTE_MODELICA.id
        });

        return this.conditions.checkModelica(menu);
    }
}
