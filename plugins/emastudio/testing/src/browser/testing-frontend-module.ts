/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { TestingMenuContribution } from "./testing-menu-contribution";
import { TestingCommandContribution } from "./testing-command-contribution";
import { TestingConditions } from "./testing-conditions";
import { CommandContribution } from "@theia/core/lib/common";
import { MenuContribution } from "@theia/core/lib/common";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bind(TestingConditions).toSelf().inSingletonScope();

    bind(TestingCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(TestingCommandContribution)
    ).inSingletonScope();

    bind(TestingMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).toDynamicValue(
        ctx => ctx.container.get(TestingMenuContribution)
    ).inSingletonScope();
});
