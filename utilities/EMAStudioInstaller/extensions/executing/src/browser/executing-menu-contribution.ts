/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MenuContribution, MenuModelRegistry } from "@theia/core/lib/common/menu";
import { CommonMenus } from "@emastudio/core/lib/browser";
import { ExecutingCommands } from "./executing-command-contribution";
import { ExecutingConditions } from "./executing-conditions";

export namespace ExecutingMenus {
    export const EXECUTING = [...CommonMenus.FEATURES, "1_executing"];
}

@injectable()
export class ExecutingMenuContribution implements MenuContribution {
    @inject(ExecutingConditions) protected readonly conditions: ExecutingConditions;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(ExecutingMenus.EXECUTING, {
            commandId: ExecutingCommands.EXECUTE.id
        });

        return this.conditions.check(menu);
    }
}
