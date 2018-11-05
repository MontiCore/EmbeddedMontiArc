/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { PATHS_PATH, PathsServer } from "../common";
import { WebSocketConnectionProvider } from "@theia/core/lib/browser/messaging";

export default new ContainerModule(bind => {
    bind(PathsServer).toDynamicValue(ctx => {
        const provider = ctx.container.get(WebSocketConnectionProvider);
        return provider.createProxy<PathsServer>(PATHS_PATH);
    }).inSingletonScope();
});
