/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MenuContribution, MenuModelRegistry } from "@theia/core/lib/common/menu";
import { CommonMenus } from "@emastudio/core/lib/browser";
import { EMAM2WASMCommands } from "./emam2wasm-command-contribution";
import { EMAM2WASMConditions } from "./emam2wasm-conditions";

export namespace EMAM2WASMMenus {
    export const EMAM2WASM = [...CommonMenus.FEATURES, "1_executing"];
}

@injectable()
export class EMAM2WASMMenuContribution implements MenuContribution {
    @inject(EMAM2WASMConditions) protected readonly conditions: EMAM2WASMConditions;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(EMAM2WASMMenus.EMAM2WASM, {
            commandId: EMAM2WASMCommands.GENERATE.id
        });

        return this.conditions.check(menu);
    }
}
