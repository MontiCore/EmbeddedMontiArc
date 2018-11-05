/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { OCLVerificationMenuContribution } from "./oclverification-menu-contribution";
import { OCLVerificationCommandContribution } from "./oclverification-command-contribution";
import { OCLVerificationConditions } from "./oclverification-conditions";
import { CommandContribution } from "@theia/core/lib/common";
import { MenuContribution } from "@theia/core/lib/common";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bind(OCLVerificationConditions).toSelf().inSingletonScope();

    bind(OCLVerificationCommandContribution).toSelf().inSingletonScope();
    bind(CommandContribution).toDynamicValue(
        ctx => ctx.container.get(OCLVerificationCommandContribution)
    ).inSingletonScope();

    bind(OCLVerificationMenuContribution).toSelf().inSingletonScope();
    bind(MenuContribution).toDynamicValue(
        ctx => ctx.container.get(OCLVerificationMenuContribution)
    ).inSingletonScope();
});
