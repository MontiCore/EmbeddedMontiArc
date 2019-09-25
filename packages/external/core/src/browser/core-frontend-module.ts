/*
 * (c) https://github.com/MontiCore/monticore
 */
import { WebSocketConnectionProvider } from "@theia/core/lib/browser";
import { MessageService } from "@theia/core/lib/common/message-service";
import { MessageClient, messageServicePath } from "@theia/core/lib/common/message-service-protocol";
import { ContainerModule } from "inversify";
import { APPLICATION_SERVER_PATH, ApplicationServer, IPCConnectionProvider } from "../common";

export default new ContainerModule((bind, unbind, isBound, rebind) => {
    bind(IPCConnectionProvider).toSelf().inSingletonScope();

    bind(ApplicationServer).toDynamicValue(
        ctx => ctx.container.get(IPCConnectionProvider).createProxy(APPLICATION_SERVER_PATH)
    ).inSingletonScope();

    rebind(MessageService).toSelf().inSingletonScope().onActivation((ctx, messages) => {
        const client = ctx.container.get(MessageClient);
        WebSocketConnectionProvider.createProxy(ctx.container, messageServicePath, client);
        IPCConnectionProvider.createProxy(ctx.container, messageServicePath, client);
        return messages;
    });
});
