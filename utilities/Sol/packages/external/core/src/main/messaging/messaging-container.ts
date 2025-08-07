/*
 * (c) https://github.com/MontiCore/monticore
 */
import { interfaces } from "inversify";
import { ConnectionHandler, RPCConnectionHandler, RPCProxyFactory } from "../../common";

import Bind = interfaces.Bind;
import Newable = interfaces.Newable;
import Abstract = interfaces.Abstract;

type ServiceIdentifier<T> = string | symbol | Newable<T> | Abstract<T>;

/*
 * These functions have been inspired by Theia's ConnectionContainerModule.
 */

export function bindFrontendService<T extends object>(bind: Bind, path: string, serviceIdentifier: ServiceIdentifier<T>) {
    const factory = new RPCProxyFactory();
    const service = factory.createProxy();

    bind<ConnectionHandler>(ConnectionHandler).toConstantValue({
        path,
        onConnection: node => factory.listen(node)
    });

    bind(serviceIdentifier).toConstantValue(service);
}

export function bindBackendService<T extends object>(bind: Bind, path: string, serviceIdentifier: ServiceIdentifier<T>) {
    bind(ConnectionHandler).toDynamicValue(ctx =>
        new RPCConnectionHandler(
            path,
            () => ctx.container.get(serviceIdentifier)
        )
    ).inSingletonScope();
}
