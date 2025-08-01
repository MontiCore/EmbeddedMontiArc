/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { createEMAStudioDashboardContainer } from "./emastudio-dashboard-container";
import { EMAStudioDashboardContribution } from "./emastudio-dashboard-contribution";
import { EMAStudioDashboardWidget } from "./emastudio-dashboard-widget";
import { CommandContribution, MenuContribution } from "@theia/core/lib/common";
import { FileSystemDashboardWidget } from "@elysium/dashboard/lib/browser/filesystem";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    bind(FileSystemDashboardWidget).toDynamicValue(
        ctx => createEMAStudioDashboardContainer(ctx.container).get(EMAStudioDashboardWidget)
    ).inSingletonScope();

    bind(EMAStudioDashboardContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(EMAStudioDashboardContribution)
    ).inSingletonScope();
    bind(MenuContribution).toDynamicValue(
        ctx => ctx.container.get(EMAStudioDashboardContribution)
    ).inSingletonScope();
});
