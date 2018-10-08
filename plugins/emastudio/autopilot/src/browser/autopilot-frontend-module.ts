/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { AutoPilotMenuContribution } from "./autopilot-menu-contribution";
import { AutoPilotCommandContribution } from "./autopilot-command-contribution";
import { AutoPilotConditions } from "./autopilot-conditions";
import { CommandContribution } from "@theia/core/lib/common";
import { MenuContribution } from "@theia/core/lib/common";
import { ExecutingHandler } from "@emastudio/executing/lib/browser";
import { AutoPilotExecutingHandler  } from "./autopilot-executing-handler";
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { AutoPilotFrontendContribution } from "./autopilot-frontend-contribution";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bind(AutoPilotConditions).toSelf().inSingletonScope();

    bind(AutoPilotExecutingHandler).toSelf().inSingletonScope();
    bind(ExecutingHandler).toDynamicValue(
        ctx => ctx.container.get(AutoPilotExecutingHandler)
    ).inSingletonScope();

    bind(AutoPilotCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(AutoPilotCommandContribution)
    ).inSingletonScope();

    bind(AutoPilotMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).toDynamicValue(
        ctx => ctx.container.get(AutoPilotMenuContribution)
    ).inSingletonScope();

    bind(AutoPilotFrontendContribution).toSelf().inSingletonScope();
    bind(FrontendApplicationContribution).toDynamicValue(
        ctx => ctx.container.get(AutoPilotFrontendContribution)
    ).inSingletonScope();
});
