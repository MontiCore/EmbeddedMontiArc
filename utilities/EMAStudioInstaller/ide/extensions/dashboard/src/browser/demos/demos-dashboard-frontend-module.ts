/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { DemosDashboardWidget } from "./demos-dashboard-widget";
import { createDemosDashboardContainer } from "./demos-dashboard-container";

export default new ContainerModule(bind => {
    bind(DemosDashboardWidget).toDynamicValue(
        ctx => createDemosDashboardContainer(ctx.container).get(DemosDashboardWidget)
    ).inSingletonScope();
});
