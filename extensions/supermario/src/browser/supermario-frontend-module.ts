/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { SuperMarioMenuContribution } from "./supermario-menu-contribution";
import { SuperMarioCommandContribution } from "./supermario-command-contribution";
import { SuperMarioConditions } from "./supermario-conditions";
import { CommandContribution, MenuContribution } from "@theia/core/lib/common";
import { ExecutingHandler } from "@emastudio/executing/lib/browser";
import { SuperMarioExecutingHandler } from "./supermario-executing-handler";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bind(SuperMarioConditions).toSelf().inSingletonScope();

    bind(SuperMarioCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(SuperMarioCommandContribution)
    ).inSingletonScope();

    bind(SuperMarioMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).toDynamicValue(
        ctx => ctx.container.get(SuperMarioMenuContribution)
    ).inSingletonScope();

    bind(SuperMarioExecutingHandler).toSelf().inSingletonScope();
    bind(ExecutingHandler).toDynamicValue(
        ctx => ctx.container.get(SuperMarioExecutingHandler)
    ).inSingletonScope();
});
