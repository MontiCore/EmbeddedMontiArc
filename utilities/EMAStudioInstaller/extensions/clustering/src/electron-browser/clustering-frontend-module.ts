/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { ClusteringFrontendContribution } from "./clustering-frontend-contribution";
import { ExecutingHandler } from "@emastudio/executing/lib/browser";
import { ClusteringExecutingHandler } from "./clustering-executing-handler";

export default new ContainerModule(bind => {
    bind(ClusteringFrontendContribution).toSelf().inSingletonScope();
    bind(FrontendApplicationContribution).toDynamicValue(
        ctx => ctx.container.get(ClusteringFrontendContribution)
    ).inSingletonScope();

    bind(ClusteringExecutingHandler).toSelf().inSingletonScope();
    bind(ExecutingHandler).toDynamicValue(
        ctx => ctx.container.get(ClusteringExecutingHandler)
    ).inSingletonScope();
});
