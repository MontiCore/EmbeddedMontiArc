/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MenuContribution, MenuModelRegistry } from "@theia/core/lib/common/menu";
import { CommonMenus } from "@emastudio/core/lib/browser";
import { VisualizationCommands } from "./visualization-command-contribution";
import { VisualizationConditions } from "./visualization-conditions";

export namespace VisualizationMenus {
    export const VISUALIZATION = [...CommonMenus.FEATURES, "2_visualization"];
}

@injectable()
export class VisualizationMenuContribution implements MenuContribution {
    @inject(VisualizationConditions) protected conditions: VisualizationConditions;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(VisualizationMenus.VISUALIZATION, {
            commandId: VisualizationCommands.VISUALIZE.id
        });

        return this.conditions.check(menu);
    }
}
