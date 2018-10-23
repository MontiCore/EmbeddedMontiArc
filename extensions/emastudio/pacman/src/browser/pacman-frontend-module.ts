/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { PacManCommandContribution } from "./pacman-command-contribution";
import { PacManMenuContribution } from "./pacman-menu-contribution";
import { PacManConditions } from "./pacman-conditions";
import { CommandContribution, MenuContribution } from "@theia/core/lib/common";
import { ExecutingHandler } from "@emastudio/executing/lib/browser";
import { PacManExecutingHandler } from "./pacman-executing-handler";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bind(PacManConditions).toSelf().inSingletonScope();

    bind(PacManCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(PacManCommandContribution)
    ).inSingletonScope();

    bind(PacManMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).toDynamicValue(
        ctx => ctx.container.get(PacManMenuContribution)
    ).inSingletonScope();

    bind(PacManExecutingHandler).toSelf().inSingletonScope();
    bind(ExecutingHandler).toDynamicValue(
        ctx => ctx.container.get(PacManExecutingHandler)
    ).inSingletonScope();
});
