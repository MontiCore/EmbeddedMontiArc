/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { ReportingMenuContribution } from "./reporting-menu-contribution";
import { ReportingCommandContribution } from "./reporting-command-contribution";
import { ReportingConditions } from "./reporting-conditions";
import { CommandContribution } from "@theia/core/lib/common";
import { MenuContribution } from "@theia/core/lib/common";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bind(ReportingConditions).toSelf().inSingletonScope();

    bind(ReportingCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(ReportingCommandContribution)
    ).inSingletonScope();

    bind(ReportingMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).toDynamicValue(
        ctx => ctx.container.get(ReportingMenuContribution)
    ).inSingletonScope();
});
