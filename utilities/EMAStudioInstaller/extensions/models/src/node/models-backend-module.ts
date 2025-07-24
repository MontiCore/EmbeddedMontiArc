/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { ModelsServerImpl } from "./models-server";
import { ModelsServer, MODELS_PATH } from "../common";
import { ConnectionHandler, JsonRpcConnectionHandler } from "@theia/core/lib/common/messaging";
import { PathContribution } from "@emastudio/paths/lib/node";
import { ModelsPathContribution } from "./models-path-contribution";

export default new ContainerModule(bind => {
    bind(ModelsServerImpl).toSelf().inSingletonScope();
    bind(ModelsServer).toService(ModelsServerImpl);
    bind(ConnectionHandler).toDynamicValue(ctx =>
        new JsonRpcConnectionHandler(MODELS_PATH, () =>
            ctx.container.get(ModelsServer)
        )
    ).inSingletonScope();

    bind(ModelsPathContribution).toSelf().inSingletonScope();
    bind(PathContribution).toDynamicValue(
        ctx => ctx.container.get(ModelsPathContribution)
    ).inSingletonScope();
});
