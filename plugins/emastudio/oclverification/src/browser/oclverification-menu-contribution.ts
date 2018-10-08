/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MenuContribution, MenuModelRegistry } from "@theia/core/lib/common/menu";
import { CommonMenus } from "@emastudio/core/lib/browser";
import { OCLVerificationCommands } from "./oclverification-command-contribution";
import { OCLVerificationConditions } from "./oclverification-conditions";

export namespace OCLVerificationMenus {
    export const OCLVERIFICATION = [...CommonMenus.FEATURES, "5_others"];
}

@injectable()
export class OCLVerificationMenuContribution implements MenuContribution {
    @inject(OCLVerificationConditions) protected readonly conditions: OCLVerificationConditions;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        await Promise.all([
            this.registerMenuCheck(registry),
            this.registerMenuVisualize(registry)
        ]);
    }

    protected async registerMenuCheck(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(OCLVerificationMenus.OCLVERIFICATION, {
            commandId: OCLVerificationCommands.CHECK.id
        });

        return this.conditions.checkCheck(menu);
    }

    protected async registerMenuVisualize(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(OCLVerificationMenus.OCLVERIFICATION, {
            commandId: OCLVerificationCommands.VISUALIZE.id
        });

        return this.conditions.checkVisualize(menu);
    }
}
