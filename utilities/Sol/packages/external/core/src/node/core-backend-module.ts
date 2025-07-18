/*
 * (c) https://github.com/MontiCore/monticore
 */
// import { BackendApplicationContribution, CliContribution } from "@theia/core/lib/node";
import { ContainerModule } from "inversify";
import { APPLICATION_CLIENT_PATH } from "../common";
import { BackendExternalBridge } from "./bridge";
import { WebSocketConnectionProvider, MessagingCliContribution } from "./messaging";

export default new ContainerModule(bind => {
    bind(WebSocketConnectionProvider).toSelf().inSingletonScope();

    bind(MessagingCliContribution).toSelf().inSingletonScope();
    // bind(CliContribution).toService(MessagingCliContribution);

    bind(BackendExternalBridge).toSelf().inSingletonScope().onActivation((ctx, client) => {
        ctx.container.get(WebSocketConnectionProvider).createProxy(APPLICATION_CLIENT_PATH, client);
        return client;
    });
    // bind(BackendApplicationContribution).toService(BackendExternalBridge);
});
