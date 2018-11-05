/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { ModelsServer, MODELS_PATH } from "../common";
import { WebSocketConnectionProvider } from "@theia/core/lib/browser/messaging";

export default new ContainerModule(bind => {
    bind(ModelsServer).toDynamicValue(ctx => {
        const provider = ctx.container.get(WebSocketConnectionProvider);
        return provider.createProxy<ModelsServer>(MODELS_PATH);
    }).inSingletonScope();
});
