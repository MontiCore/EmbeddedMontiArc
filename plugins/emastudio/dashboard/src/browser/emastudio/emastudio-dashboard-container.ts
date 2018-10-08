/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { interfaces, Container } from "inversify";
import { EMAStudioDashboard } from "./emastudio-dashboard";
import { EMAStudioDashboardWidget } from "./emastudio-dashboard-widget";
import { EMAStudioDashboardModel } from "./emastudio-dashboard-model";
import { BaseDashboardModel, DashboardModel } from "@elysium/dashboard/lib/browser/dashboard-model";
import { BaseDashboardWidget, DashboardProps } from "@elysium/dashboard/lib/browser/dashboard-widget";
import { createDashboardContainer } from "@elysium/dashboard/lib/browser/dashboard-container";
import { BaseDashboard, Dashboard } from "@elysium/dashboard/lib/browser/dashboard";
import { FILESYSTEM_DASHBOARD_PROPS } from "@elysium/dashboard/lib/browser/filesystem/filesystem-dashboard-container";
import {
    FileSystemDashboardQueue, FileSystemDashboardModel, FileSystemDashboard, FileSystemDashboardWidget
} from "@elysium/dashboard/lib/browser/filesystem";

export function createEMAStudioDashboardContainer(parent: interfaces.Container): Container {
    const child = createDashboardContainer(parent);

    child.unbind(BaseDashboard);
    child.bind(EMAStudioDashboard).toSelf();
    child.bind(FileSystemDashboard).to(EMAStudioDashboard);
    child.rebind(Dashboard).toDynamicValue(
        ctx => ctx.container.get(FileSystemDashboard)
    );

    child.unbind(BaseDashboardModel);
    child.bind(EMAStudioDashboardModel).toSelf();
    child.bind(FileSystemDashboardModel).to(EMAStudioDashboardModel);
    child.rebind(DashboardModel).toDynamicValue(
        ctx => ctx.container.get(FileSystemDashboardModel)
    );

    child.unbind(BaseDashboardWidget);
    child.bind(EMAStudioDashboardWidget).toSelf();
    child.bind(FileSystemDashboardWidget).to(EMAStudioDashboardWidget);

    child.rebind(DashboardProps).toConstantValue(FILESYSTEM_DASHBOARD_PROPS);

    child.bind(FileSystemDashboardQueue).toSelf().inSingletonScope();

    return child;
}
