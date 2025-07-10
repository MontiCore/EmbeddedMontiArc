/*
 * (c) https://github.com/MontiCore/monticore
 */
import { bindContributionProvider } from "@theia/core";
import { TabBarToolbarContribution } from "@theia/core/lib/browser/shell/tab-bar-toolbar";
import { Container, ContainerModule } from "inversify";
import { bindCommon } from "../common/configuration-common-module";
import { ConfigurationDialog, ConfigurationDialogFactory, ConfigurationDialogProps } from "./configuration-dialog";
import { ConfigurationManager, ConfigurationManagerImpl } from "./configuration-manager";
import { ConfigurationViewContribution } from "./configuration-view-contribution";
import { ConfigurationWidget } from "./configuration-widget";
import { ConfigurationRunnerClientImpl, FrontendConfigurationRunnerRegistryImpl } from "./runner";

import {
    ConfigurationProcessorBuffer,
    ConfigurationProcessorBufferImpl,
    ConfigurationProcessorSelectionService,
    ConfigurationProcessorSelectionServiceImpl, ConfigurationProcessorService, ConfigurationProcessorServiceImpl,
    ConfigurationProcessorViewContribution,
    ConfigurationProcessorWidget
} from "./processor";

import {
    CONFIGURATION_PROCESSOR_PATH, CONFIGURATION_RUNNER_PATH,
    ConfigurationProcessorServer, ConfigurationProcessorWatcher,
    ConfigurationRunnerClient,
    ConfigurationRunnerRegistry,
} from "../common";

import {
    bindViewContribution,
    FrontendApplicationContribution,
    WebSocketConnectionProvider,
    WidgetFactory
} from "@theia/core/lib/browser";

import {
    ConfigurationTypeContribution,
    ConfigurationTypeRegistry,
    ConfigurationTypeRegistryImpl
} from "./configuration-type-registry";

import "../../src/browser/style/configurations.css";

export default new ContainerModule(bind => {
    bindCommon(bind);
    bindViewContribution(bind, ConfigurationViewContribution);
    bindViewContribution(bind, ConfigurationProcessorViewContribution);
    bindContributionProvider(bind, ConfigurationTypeContribution);

    bind(TabBarToolbarContribution).toService(ConfigurationViewContribution);
    bind(FrontendApplicationContribution).toService(ConfigurationProcessorViewContribution);

    bind(ConfigurationTypeRegistryImpl).toSelf().inSingletonScope();
    bind(ConfigurationTypeRegistry).toService(ConfigurationTypeRegistryImpl);
    bind(FrontendApplicationContribution).toService(ConfigurationTypeRegistryImpl);

    bind(ConfigurationManagerImpl).toSelf().inSingletonScope();
    bind(ConfigurationManager).toService(ConfigurationManagerImpl);
    bind(FrontendApplicationContribution).toService(ConfigurationManagerImpl);

    bind(ConfigurationProcessorBuffer).to(ConfigurationProcessorBufferImpl).inSingletonScope();
    bind(FrontendApplicationContribution).toService(ConfigurationProcessorBuffer);

    bind(ConfigurationProcessorSelectionService).to(ConfigurationProcessorSelectionServiceImpl).inSingletonScope();

    bind(ConfigurationProcessorService).to(ConfigurationProcessorServiceImpl).inSingletonScope();

    bind(FrontendConfigurationRunnerRegistryImpl).toSelf().inSingletonScope();
    bind(ConfigurationRunnerRegistry).toService(FrontendConfigurationRunnerRegistryImpl);
    bind(FrontendApplicationContribution).toService(FrontendConfigurationRunnerRegistryImpl);

    bind(ConfigurationDialogFactory).toFactory(ctx =>
        (props: ConfigurationDialogProps) => {
            const container = new Container();

            container.parent = ctx.container;

            container.bind(ConfigurationDialog).toSelf();
            container.bind(ConfigurationDialogProps).toConstantValue(props);

            return container.get(ConfigurationDialog);
        }
    );

    bind(ConfigurationWidget).toSelf();
    bind(WidgetFactory).toDynamicValue(ctx => ({
        id: ConfigurationWidget.ID,
        createWidget: () => ctx.container.get(ConfigurationWidget)
    }));

    bind(ConfigurationProcessorWidget).toSelf();
    bind(WidgetFactory).toDynamicValue(ctx => ({
        id: ConfigurationProcessorWidget.ID,
        createWidget: () => ctx.container.get(ConfigurationProcessorWidget)
    }));

    bind(ConfigurationProcessorServer).toDynamicValue(ctx => {
        const provider = ctx.container.get(WebSocketConnectionProvider);
        const watcher = ctx.container.get(ConfigurationProcessorWatcher);

        return provider.createProxy(CONFIGURATION_PROCESSOR_PATH, watcher.getConfigurationProcessorClient());
    }).inSingletonScope();

    bind(ConfigurationRunnerClient).to(ConfigurationRunnerClientImpl).inSingletonScope().onActivation((ctx, client) => {
        WebSocketConnectionProvider.createProxy(ctx.container, CONFIGURATION_RUNNER_PATH, client as ConfigurationRunnerClient);
        return client;
    });
    bind(FrontendApplicationContribution).toService(ConfigurationRunnerClient);
});
