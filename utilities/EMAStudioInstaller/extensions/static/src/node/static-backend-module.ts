/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { bindContributionProvider } from "@theia/core/lib/common";
import { StaticContribution, StaticRegistry } from "./static-registry";
import { BackendApplicationContribution } from "@theia/core/lib/node";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, StaticContribution);

    bind(StaticRegistry).toSelf().inSingletonScope();
    bind(BackendApplicationContribution).toDynamicValue(
        ctx => ctx.container.get(StaticRegistry)
    ).inSingletonScope();
});
