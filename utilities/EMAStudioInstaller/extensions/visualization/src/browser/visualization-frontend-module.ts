/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { VisualizationMenuContribution } from "./visualization-menu-contribution";
import { VisualizationCommandContribution } from "./visualization-command-contribution";
import { VisualizationConditions } from "./visualization-conditions";
import { CommandContribution } from "@theia/core/lib/common";
import { MenuContribution } from "@theia/core/lib/common";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bind(VisualizationConditions).toSelf().inSingletonScope();

    bind(VisualizationCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(VisualizationCommandContribution)
    ).inSingletonScope();

    bind(VisualizationMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).toDynamicValue(
        ctx => ctx.container.get(VisualizationMenuContribution)
    ).inSingletonScope();
});
