/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { ExecutingMenuContribution } from "./executing-menu-contribution";
import { ExecutingCommandContribution } from "./executing-command-contribution";
import { ExecutingConditions } from "./executing-conditions";
import { CommandContribution } from "@theia/core/lib/common";
import { MenuContribution } from "@theia/core/lib/common";
import { bindContributionProvider } from "@theia/core/lib/common";
import { ExecutingHandler, DefaultExecutingHandler } from "./executing-handler";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, ExecutingHandler);

    bind(DefaultExecutingHandler).toSelf().inSingletonScope();

    bind(ExecutingConditions).toSelf().inSingletonScope();

    bind(ExecutingCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(ExecutingCommandContribution)
    ).inSingletonScope();

    bind(ExecutingMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).toDynamicValue(
        ctx => ctx.container.get(ExecutingMenuContribution)
    ).inSingletonScope();
});
