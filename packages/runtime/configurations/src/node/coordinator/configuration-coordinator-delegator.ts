/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationToken } from "@theia/core/lib/common";
import { inject, injectable } from "inversify";
import { Configuration } from "../../common";
import { ConfigurationCoordinatorRegistry } from "./configuration-coordinator-registry";

export const ConfigurationCoordinatorDelegator = Symbol("ConfigurationCoordinatorDelegator");
export interface ConfigurationCoordinatorDelegator {
    run<V>(configuration: Configuration<V>, context: OptionsContext, token: CancellationToken): Promise<void>;
}

@injectable()
export class ConfigurationCoordinatorDelegatorImpl implements ConfigurationCoordinatorDelegator {
    @inject(ConfigurationCoordinatorRegistry) protected readonly registry: ConfigurationCoordinatorRegistry;

    public async run<V>(configuration: Configuration<V>, context: OptionsContext, token: CancellationToken): Promise<void> {
        const coordinator = this.registry.getCoordinator(configuration.typeId);

        if (coordinator) return coordinator.run(configuration.uuid, configuration.options, context, token);
        else console.warn(`There is no coordinator with id '${configuration.typeId}'.`);
    }
}
