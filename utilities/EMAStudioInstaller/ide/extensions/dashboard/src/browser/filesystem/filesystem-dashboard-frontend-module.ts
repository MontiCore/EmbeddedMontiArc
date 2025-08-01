/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { FileSystemDashboardWidget } from "./filesystem-dashboard-widget";
import { createFileSystemDashboardContainer } from "./filesystem-dashboard-container";
import { CommandContribution, MenuContribution } from "@theia/core/lib/common";
import { FileSystemDashboardContribution } from "./filesystem-dashboard-contribution";

export default new ContainerModule(bind => {
    bind(FileSystemDashboardWidget).toDynamicValue(
        ctx => createFileSystemDashboardContainer(ctx.container).get(FileSystemDashboardWidget)
    ).inSingletonScope();

    bind(FileSystemDashboardContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(FileSystemDashboardContribution)
    ).inSingletonScope();
    bind(MenuContribution).toDynamicValue(
        ctx => ctx.container.get(FileSystemDashboardContribution)
    ).inSingletonScope();
});
