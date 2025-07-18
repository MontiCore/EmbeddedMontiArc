/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MenuContribution, MenuModelRegistry } from "@theia/core/lib/common/menu";
import { CommonMenus } from "@emastudio/core/lib/browser";
import { TestingCommands } from "./testing-command-contribution";
import { TestingConditions } from "./testing-conditions";

export namespace TestingMenus {
    export const TESTING = [...CommonMenus.FEATURES, "4_testing"];
}

@injectable()
export class TestingMenuContribution implements MenuContribution {
    @inject(TestingConditions) protected readonly conditions: TestingConditions;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        await Promise.all([
            this.registerMenu(registry),
            this.registerMenuAll(registry)
        ]);
    }

    protected async registerMenu(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(TestingMenus.TESTING, {
            commandId: TestingCommands.TEST.id
        });

        return this.conditions.check(menu);
    }

    protected async registerMenuAll(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(TestingMenus.TESTING, {
            commandId: TestingCommands.TEST_ALL.id
        });

        return this.conditions.checkAll(menu);
    }
}
