/*
 * (c) https://github.com/MontiCore/monticore
 */
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { ContainerModule, interfaces } from "inversify";
import { bindCommon } from "../common/configuration-common-module";
import { ConfigurationProcessor, ConfigurationProcessorImpl, ConfigurationProcessorServerImpl } from "./processor";
import { BackendConfigurationRunnerRegistryImpl } from "./runner";

import {
    bindContributionProvider,
    ConnectionHandler,
    JsonRpcConnectionHandler, JsonRpcProxyFactory
} from "@theia/core/lib/common";

import {
    CONFIGURATION_PROCESSOR_PATH,
    CONFIGURATION_RUNNER_PATH, ConfigurationProcessorClient, ConfigurationProcessorServer,
    ConfigurationRunnerClient,
    ConfigurationRunnerRegistry
} from "../common";

import {
    ConfigurationCoordinatorContribution,
    ConfigurationCoordinatorDelegator,
    ConfigurationCoordinatorDelegatorImpl, ConfigurationCoordinatorRegistry, ConfigurationCoordinatorRegistryImpl
} from "./coordinator";

import Bind = interfaces.Bind;

function bindConfigurationRunnerClient(bind: Bind): void {
    const factory = new JsonRpcProxyFactory();
    const service = factory.createProxy();

    bind<ConnectionHandler>(ConnectionHandler).toConstantValue({
        path: CONFIGURATION_RUNNER_PATH,
        onConnection: connection => factory.listen(connection)
    });

    bind(ConfigurationRunnerClient).toConstantValue(service);
}

export default new ContainerModule(bind => {
    bindCommon(bind);
    bindContributionProvider(bind, ConfigurationCoordinatorContribution);
    bindConfigurationRunnerClient(bind);

    bind(BackendConfigurationRunnerRegistryImpl).toSelf().inSingletonScope();
    bind(ConfigurationRunnerRegistry).toService(BackendConfigurationRunnerRegistryImpl);
    bind(BackendApplicationContribution).toService(BackendConfigurationRunnerRegistryImpl);

    bind(ConfigurationCoordinatorDelegator).to(ConfigurationCoordinatorDelegatorImpl).inSingletonScope();

    bind(ConfigurationCoordinatorRegistryImpl).toSelf().inSingletonScope();
    bind(ConfigurationCoordinatorRegistry).toService(ConfigurationCoordinatorRegistryImpl);
    bind(BackendApplicationContribution).toService(ConfigurationCoordinatorRegistryImpl);

    bind(ConfigurationProcessor).to(ConfigurationProcessorImpl).inSingletonScope();

    bind(ConfigurationProcessorServer).to(ConfigurationProcessorServerImpl).inSingletonScope();
    bind(ConnectionHandler).toDynamicValue(ctx =>
        new JsonRpcConnectionHandler<ConfigurationProcessorClient>(
            CONFIGURATION_PROCESSOR_PATH,
            client => {
                const server = ctx.container.get<ConfigurationProcessorServer>(ConfigurationProcessorServer);

                server.setClient(client);
                client.onDidCloseConnection(() => server.unsetClient(client));
                return server;
            }
        )
    ).inSingletonScope();
});
