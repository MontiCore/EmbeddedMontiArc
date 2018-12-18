/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { InteractiveSimulatorMenuContribution } from "./interactivesimulator-menu-contribution";
import { InteractiveSimulatorCommandContribution } from "./interactivesimulator-command-contribution";
import { InteractiveSimulatorConditions } from "./interactivesimulator-conditions";
import { CommandContribution } from "@theia/core/lib/common";
import { MenuContribution } from "@theia/core/lib/common";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bind(InteractiveSimulatorConditions).toSelf().inSingletonScope();

    bind(InteractiveSimulatorCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(InteractiveSimulatorCommandContribution)
    ).inSingletonScope();

    bind(InteractiveSimulatorMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).toDynamicValue(
        ctx => ctx.container.get(InteractiveSimulatorMenuContribution)
    ).inSingletonScope();
});
