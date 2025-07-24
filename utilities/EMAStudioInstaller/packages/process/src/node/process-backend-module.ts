/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { ContainerModule } from "inversify";
import { MessagingService } from "@theia/core/lib/node/messaging";
import { ConnectionHandler, JsonRpcConnectionHandler } from "@theia/core/lib/common/messaging";
import { PROCESS_PATH, ProcessServer } from "../common/process-protocol";
import { ProcessServerImpl } from "./process-server";
import { ProcessMessagingService } from "./process-messaging-service";

import MessagingServiceContribution = MessagingService.Contribution;

export default new ContainerModule(bind => {
    bind(ProcessMessagingService).toSelf().inSingletonScope();
    bind(MessagingServiceContribution).to(ProcessMessagingService).inSingletonScope();

    bind(ProcessServerImpl).toSelf().inSingletonScope();
    bind(ProcessServer).toService(ProcessServerImpl);
    bind(ConnectionHandler).toDynamicValue(
        ctx => new JsonRpcConnectionHandler(PROCESS_PATH, () =>
            ctx.container.get(ProcessServer)
        )
    ).inSingletonScope();
});
