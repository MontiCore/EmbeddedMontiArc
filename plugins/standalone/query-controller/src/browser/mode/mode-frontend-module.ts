/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { bindContributionProvider } from "@theia/core/lib/common";
import { ModeQueryHandler, ModeController } from "./mode-controller";
import { StaticModeQueryHandler } from "./static-mode-query-handler";
import { QueryHandler } from "../query-controller";
import { LoadModeQueryHandler } from "./load-mode-query-handler";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, ModeQueryHandler);

    bind(ModeController).toSelf().inSingletonScope();
    bind(QueryHandler).toDynamicValue(
        ctx => ctx.container.get(ModeController)
    ).inSingletonScope();

    bind(StaticModeQueryHandler).toSelf().inSingletonScope();
    bind(ModeQueryHandler).toDynamicValue(
        ctx => ctx.container.get(StaticModeQueryHandler)
    ).inSingletonScope();

    bind(LoadModeQueryHandler).toSelf().inSingletonScope();
    bind(ModeQueryHandler).toDynamicValue(
        ctx => ctx.container.get(LoadModeQueryHandler)
    ).inSingletonScope();
});
