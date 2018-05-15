/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { bindContributionProvider } from "@theia/core/lib/common";
import { MultiModeContribution, MultiModeProcessor } from "./multi-mode";
import { StaticMultiModeContribution } from "./static-multi-mode-contribution";
import { FrontendApplicationContribution } from "@theia/core/lib/browser";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, MultiModeContribution);

    bind(MultiModeProcessor).toSelf().inSingletonScope();
    bind(FrontendApplicationContribution).toDynamicValue(
        ctx => ctx.container.get(MultiModeProcessor)
    ).inSingletonScope();

    bind(StaticMultiModeContribution).toSelf().inSingletonScope();
    bind(MultiModeContribution).toDynamicValue(
        ctx => ctx.container.get(StaticMultiModeContribution)
    ).inSingletonScope();
});
