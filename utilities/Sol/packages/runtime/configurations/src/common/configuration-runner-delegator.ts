/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationToken } from "@theia/core/lib/common";
import { inject, injectable } from "inversify";
import { ConfigurationRunnerRegistry } from "./configuration-runner-registry";

export const ConfigurationRunnerDelegator = Symbol("ConfigurationRunnerDelegator");
export interface ConfigurationRunnerDelegator {
    run<V>(uuid: string, typeId: string, taskName: string, options: V, context: OptionsContext, token: CancellationToken): Promise<void>;
}

@injectable()
export class ConfigurationRunnerDelegatorImpl implements ConfigurationRunnerDelegator {
    @inject(ConfigurationRunnerRegistry) protected readonly registry: ConfigurationRunnerRegistry;

    public async run<V>(uuid: string, typeId: string, taskName: string, options: V, context: OptionsContext, token: CancellationToken): Promise<void> {
        const runner = this.registry.getRunner(typeId);

        if (runner) return runner.run(uuid, taskName, options, context, token);
        else console.warn(`There is no runner with id '${typeId}'.`);
    }
}
