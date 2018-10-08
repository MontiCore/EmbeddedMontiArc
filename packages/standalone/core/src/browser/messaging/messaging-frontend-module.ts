/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { WorkerConnectionProvider } from "./connection";
import { WebSocketConnectionProvider } from "@theia/core/lib/browser/messaging/ws-connection-provider";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    bind(WorkerConnectionProvider).toSelf().inSingletonScope();

    // tslint:disable-next-line:no-any
    rebind(WebSocketConnectionProvider).toConstantValue(undefined as any);
});
