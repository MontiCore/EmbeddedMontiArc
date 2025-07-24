/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { bindContributionProvider } from "@theia/core/lib/common";
import { QueryController, QueryHandler } from "./query-controller";
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { OpenFileQueryHandler } from "./open-file-query-handler";
import { HideControlsQueryHandler } from "./hide-controls-query-handler";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, QueryHandler);

    bind(QueryController).toSelf().inSingletonScope();
    bind(FrontendApplicationContribution).toDynamicValue(
        ctx => ctx.container.get(QueryController)
    ).inSingletonScope();

    bind(OpenFileQueryHandler).toSelf().inSingletonScope();
    bind(QueryHandler).toDynamicValue(
        ctx => ctx.container.get(OpenFileQueryHandler)
    ).inSingletonScope();

    bind(HideControlsQueryHandler).toSelf().inSingletonScope();
    bind(QueryHandler).toDynamicValue(
        ctx => ctx.container.get(HideControlsQueryHandler)
    ).inSingletonScope();
});
