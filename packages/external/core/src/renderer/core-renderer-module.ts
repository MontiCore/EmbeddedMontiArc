/*
 * (c) https://github.com/MontiCore/monticore
 */
import { MessageClient, messageServicePath } from "@theia/core/lib/common/message-service-protocol";
import { bindContributionProvider } from "@theia/core/lib/common/contribution-provider";
import { ContainerModule } from "inversify";
import { HISTORY_PATH, HistoryClient } from "../common/history";
import { IPCConnectionProvider } from "../common/messaging";
import { Application, RendererApplication } from "./application";
import { DialogService, DialogServiceImpl } from "./dialog";
import { HistoryClientImpl } from "./history/history-client-impl";
import { NotificationsClientImpl } from "./messages";
import { RouteContribution } from "./router-fragment";
import { APPLICATION_SERVER_PATH, ApplicationServer, DIALOG_PATH, DialogServer, APPLICATION_CLIENT_PATH } from "../common";

import "../../src/renderer/style/index.css";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, RouteContribution);

    bind(IPCConnectionProvider).toSelf().inSingletonScope();

    bind(NotificationsClientImpl).toSelf().inSingletonScope();
    bind(MessageClient).toDynamicValue(
        ctx => ctx.container.get(NotificationsClientImpl)
    ).inSingletonScope().onActivation((context, client: MessageClient) => {
        context.container.get(IPCConnectionProvider).createProxy(messageServicePath, client);
        return client;
    });

    bind(HistoryClientImpl).toSelf().inSingletonScope();
    bind(HistoryClient).toDynamicValue(
        ctx => ctx.container.get(HistoryClientImpl)
    ).inSingletonScope().onActivation((context, client: HistoryClient) => {
        context.container.get(IPCConnectionProvider).createProxy(HISTORY_PATH, client);
        return client;
    });

    bind(RendererApplication).toSelf().inSingletonScope().onActivation((context, client: RendererApplication) => {
        context.container.get(IPCConnectionProvider).createProxy(APPLICATION_CLIENT_PATH, client);
        return client;
    });
    bind(Application).toService(RendererApplication);

    bind(ApplicationServer).toDynamicValue(
        ctx => ctx.container.get(IPCConnectionProvider).createProxy(APPLICATION_SERVER_PATH)
    ).inSingletonScope();

    bind(DialogServer).toDynamicValue(
        ctx => ctx.container.get(IPCConnectionProvider).createProxy(DIALOG_PATH)
    ).inSingletonScope();

    bind(DialogServiceImpl).toSelf().inSingletonScope();
    bind(DialogService).toService(DialogServiceImpl);
});
