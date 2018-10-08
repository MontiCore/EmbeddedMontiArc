/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { interfaces, Container } from "inversify";
import { BaseDashboard, Dashboard } from "./dashboard";
import { DashboardSelectionService, DashboardSelectionServiceImpl } from "./dashboard-selection";
import { BaseDashboardModel, DashboardModel } from "./dashboard-model";
import { BaseDashboardWidget, DashboardProps } from "./dashboard-widget";

import "../../src/browser/style/dashboard.css";
import "../../src/browser/style/icons-sprite.css";

export function createDashboardContainer(parent: interfaces.Container): Container {
    const child = new Container({ defaultScope: "Singleton" });

    child.parent = parent;

    child.bind(BaseDashboard).toSelf();
    child.bind(Dashboard).toDynamicValue(ctx => ctx.container.get(BaseDashboard));

    child.bind(DashboardSelectionServiceImpl).toSelf();
    child.bind(DashboardSelectionService).toDynamicValue(ctx => ctx.container.get(DashboardSelectionServiceImpl));

    child.bind(BaseDashboardModel).toSelf();
    child.bind(DashboardModel).toDynamicValue(ctx => ctx.container.get(BaseDashboardModel));

    child.bind(BaseDashboardWidget).toSelf();
    child.bind(DashboardProps).toConstantValue({ title: "Dashboard" });

    return child;
}
