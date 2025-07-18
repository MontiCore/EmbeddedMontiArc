/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { BaseDashboard, Dashboard } from "../dashboard";
import { BaseDashboardModel, DashboardModel } from "../dashboard-model";
import { createDashboardContainer } from "../dashboard-container";
import { Container, interfaces } from "inversify";
import { DashboardProps, BaseDashboardWidget } from "../dashboard-widget";
import { DemosDashboard } from "./demos-dashboard";
import { DemosDashboardModel } from "./demos-dashboard-model";
import { DemosDashboardWidget } from "./demos-dashboard-widget";

export const DEMOS_DASHBOARD_PROPS = <DashboardProps> {
    title: "Demos"
};

export function createDemosDashboardContainer(parent: interfaces.Container): Container {
    const child = createDashboardContainer(parent);

    child.unbind(BaseDashboard);
    child.bind(DemosDashboard).toSelf();
    child.rebind(Dashboard).toDynamicValue(ctx => ctx.container.get(DemosDashboard));

    child.unbind(BaseDashboardModel);
    child.bind(DemosDashboardModel).toSelf();
    child.rebind(DashboardModel).toDynamicValue(ctx => ctx.container.get(DemosDashboardModel));

    child.unbind(BaseDashboardWidget);
    child.bind(DemosDashboardWidget).toSelf();

    child.rebind(DashboardProps).toConstantValue(DEMOS_DASHBOARD_PROPS);

    return child;
}
