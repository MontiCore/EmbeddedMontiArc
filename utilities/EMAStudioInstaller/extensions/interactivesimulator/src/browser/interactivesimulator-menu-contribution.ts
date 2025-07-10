/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MenuContribution, MenuModelRegistry } from "@theia/core/lib/common/menu";
import { CommonMenus } from "@emastudio/core/lib/browser";
import { InteractiveSimulatorCommands } from "./interactivesimulator-command-contribution";
import { InteractiveSimulatorConditions } from "./interactivesimulator-conditions";

export namespace InteractiveSimulatorMenus {
    export const INTERACTIVESIMULATOR = [...CommonMenus.FEATURES, "5_others"];
}

@injectable()
export class InteractiveSimulatorMenuContribution implements MenuContribution {
    @inject(InteractiveSimulatorConditions) protected readonly conditions: InteractiveSimulatorConditions;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        await Promise.all([
            this.registerMenuDebug(registry),
            this.registerMenuDebugWosvg(registry)
        ]);
    }

    protected async registerMenuDebug(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(InteractiveSimulatorMenus.INTERACTIVESIMULATOR, {
            commandId: InteractiveSimulatorCommands.DEBUG.id
        });

        return this.conditions.checkDebug(menu);
    }

    protected async registerMenuDebugWosvg(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(InteractiveSimulatorMenus.INTERACTIVESIMULATOR, {
            commandId: InteractiveSimulatorCommands.DEBUG_WOSVG.id
        });

        return this.conditions.checkDebugWosvg(menu);
    }
}
