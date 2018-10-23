/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MenuContribution, MenuModelRegistry } from "@theia/core/lib/common/menu";
import { CommonMenus } from "@emastudio/core/lib/browser";
import { SuperMarioCommands } from "./supermario-command-contribution";
import { SuperMarioConditions } from "./supermario-conditions";

export namespace SuperMarioMenus {
    export const SUPERMARIO = [...CommonMenus.FEATURES, "5_others"];
}

@injectable()
export class SuperMarioMenuContribution implements MenuContribution {
    @inject(SuperMarioConditions) protected readonly conditions: SuperMarioConditions;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(SuperMarioMenus.SUPERMARIO, {
            commandId: SuperMarioCommands.PLAY.id
        });

        return this.conditions.check(menu);
    }
}
