/*
 * (c) https://github.com/MontiCore/monticore
 */
import { MessageClient, messageServicePath } from "@theia/core/lib/common/message-service-protocol";
import { bindContributionProvider } from "@theia/core/lib/common/contribution-provider";
import { ContainerModule } from "inversify";
import { ConnectionHandler } from "../common/messaging";
import { Application, ApplicationContribution, ApplicationPhase, MainApplication } from "./application";
import { BinaryService, BinaryServiceImpl } from "./binary";
import { DialogServerImpl } from "./dialog";
import { NotificationsService, NotificationsServiceImpl } from "./messages";
import {
    MessagingContribution,
    bindFrontendService,
    bindBackendService,
    IPCMainTransportFactory,
    IPCMainTransport
} from "./messaging";
import { StorageService, StorageServiceImpl } from "./storage/storage-service";
import { WindowService, WindowServiceImpl } from "./window/window-service";

import {
    APPLICATION_SERVER_PATH, ApplicationClient, ApplicationServer, DIALOG_PATH, DialogServer,
    APPLICATION_CLIENT_PATH
} from "../common";

export default new ContainerModule(bind => {
    bindContributionProvider(bind, ConnectionHandler);
    bindContributionProvider(bind, ApplicationContribution);
    bindContributionProvider(bind, ApplicationPhase);

    bind(MainApplication).toSelf().inSingletonScope();
    bind(Application).toService(MainApplication);
    bind(ApplicationServer).toService(MainApplication);

    bind(WindowServiceImpl).toSelf().inSingletonScope();
    bind(WindowService).toService(WindowServiceImpl);

    bind(BinaryServiceImpl).toSelf().inSingletonScope();
    bind(BinaryService).toService(BinaryServiceImpl);

    bind(NotificationsServiceImpl).toSelf().inSingletonScope();
    bind(NotificationsService).toService(NotificationsServiceImpl);

    bind(MessagingContribution).toSelf().inSingletonScope();
    bind(ApplicationContribution).toService(MessagingContribution);

    bind(StorageServiceImpl).toSelf().inSingletonScope();
    bind(StorageService).toService(StorageServiceImpl);

    bind(DialogServerImpl).toSelf().inSingletonScope();
    bind(DialogServer).toService(DialogServerImpl);

    bind(IPCMainTransportFactory).toFactory(ctx =>
        (path: string) => {
            const windows = ctx.container.get<WindowService>(WindowService);

            return new IPCMainTransport(path, windows);
        }
    );

    bindFrontendService(bind, messageServicePath, MessageClient);
    bindFrontendService(bind, APPLICATION_CLIENT_PATH, ApplicationClient);

    bindBackendService(bind, APPLICATION_SERVER_PATH, ApplicationServer);
    bindBackendService(bind, DIALOG_PATH, DialogServer);
});
