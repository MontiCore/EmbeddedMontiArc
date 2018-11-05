/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable, inject } from "inversify";
import { MenuContribution, MenuModelRegistry } from "@theia/core/lib/common/menu";
import { CommonMenus } from "@emastudio/core/lib/browser";
import { ReportingCommands } from "./reporting-command-contribution";
import { ReportingConditions } from "./reporting-conditions";

export namespace ReportingMenus {
    export const REPORTING = [...CommonMenus.FEATURES, "3_reporting"];
}

@injectable()
export class ReportingMenuContribution implements MenuContribution {
    @inject(ReportingConditions) protected readonly conditions: ReportingConditions;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        await Promise.all([
            this.registerMenu(registry),
            this.registerMenuStreams(registry)
        ]);
    }

    protected async registerMenu(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(ReportingMenus.REPORTING, {
            commandId: ReportingCommands.REPORT.id
        });

        return this.conditions.check(menu);
    }

    protected async registerMenuStreams(registry: MenuModelRegistry): Promise<void> {
        const menu = registry.registerMenuAction(ReportingMenus.REPORTING, {
            commandId: ReportingCommands.REPORT_STREAMS.id
        });

        return this.conditions.checkStreams(menu);
    }
}
