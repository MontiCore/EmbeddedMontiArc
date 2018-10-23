/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MenuContribution, MenuModelRegistry } from "@theia/core/lib/common/menu";
import { CommonMenus } from "@emastudio/core/lib/browser";
import { PacManCommands } from "./pacman-command-contribution";
import { PacManConditions } from "./pacman-conditions";

export namespace PacManMenus {
    export const PACMAN = [...CommonMenus.FEATURES, "5_others"];
}

@injectable()
export class PacManMenuContribution implements MenuContribution {
    @inject(PacManConditions) protected readonly conditions: PacManConditions;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(PacManMenus.PACMAN, {
            commandId: PacManCommands.PLAY.id
        });

        return this.conditions.check(menu);
    }
}
