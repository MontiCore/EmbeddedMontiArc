/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { createDashboardContainer } from "../dashboard-container";
import { interfaces, Container } from "inversify";
import { BaseDashboard, Dashboard } from "../dashboard";
import { FileSystemDashboard } from "./filesystem-dashboard";
import { BaseDashboardModel, DashboardModel } from "../dashboard-model";
import { FileSystemDashboardModel } from "./filesystem-dashboard-model";
import { BaseDashboardWidget, DashboardProps } from "../dashboard-widget";
import { FileSystemDashboardWidget } from "./filesystem-dashboard-widget";
import { CONTROL_CONTEXT_MENU, ITEM_CONTEXT_MENU } from "./filesystem-dashboard-contribution";
import { FileSystemDashboardQueue } from "./filesystem-dashboard-queue";

export const FILESYSTEM_DASHBOARD_PROPS = <DashboardProps> {
    title: "Dashboard",
    controlContextMenu: CONTROL_CONTEXT_MENU,
    itemContextMenu: ITEM_CONTEXT_MENU
};

export function createFileSystemDashboardContainer(parent: interfaces.Container): Container {
    const child = createDashboardContainer(parent);

    child.unbind(BaseDashboard);
    child.bind(FileSystemDashboard).toSelf();
    child.rebind(Dashboard).toDynamicValue(
        ctx => ctx.container.get(FileSystemDashboard)
    );

    child.unbind(BaseDashboardModel);
    child.bind(FileSystemDashboardModel).toSelf();
    child.rebind(DashboardModel).toDynamicValue(
        ctx => ctx.container.get(FileSystemDashboardModel)
    );

    child.unbind(BaseDashboardWidget);
    child.bind(FileSystemDashboardWidget).toSelf();

    child.rebind(DashboardProps).toConstantValue(FILESYSTEM_DASHBOARD_PROPS);

    child.bind(FileSystemDashboardQueue).toSelf().inSingletonScope();

    return child;
}
