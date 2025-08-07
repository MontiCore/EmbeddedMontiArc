/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ConnectionHandler, JsonRpcConnectionHandler } from "@theia/core/lib/common";
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { ContainerModule } from "inversify";
import { bindCommon } from "../common/options-common-module";
import { BackendValidatorRegistry } from "./backend-validator-registry";
import { ValidatorServerImpl } from "./validator-server-impl";
import { ValidatorPaths, ValidatorServer, ValidatorRegistry } from "../common";

export default new ContainerModule(bind => {
    bindCommon(bind);

    bind(BackendValidatorRegistry).toSelf().inSingletonScope();
    bind(ValidatorRegistry).toService(BackendValidatorRegistry);
    bind(BackendApplicationContribution).toService(BackendValidatorRegistry);

    bind(ValidatorServerImpl).toSelf().inSingletonScope();
    bind(ValidatorServer).toService(ValidatorServerImpl);
    bind(ConnectionHandler).toDynamicValue(ctx =>
        new JsonRpcConnectionHandler(
            ValidatorPaths.PATH,
            () => ctx.container.get(ValidatorServer)
        )
    ).inSingletonScope();
});
