/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationToken } from "@theia/core/lib/common";
import { inject, injectable, unmanaged } from "inversify";
import { ConfigurationRunnerClient } from "../../common";
import { ConfigurationRunnerDelegator } from "../../common";

export interface ConfigurationCoordinator<V = any> { // tslint:disable-line:no-any
    readonly id: string;

    run(uuid: string, options: V, context: OptionsContext, token: CancellationToken): Promise<void>;
}

@injectable()
export abstract class CommonConfigurationCoordinator<V = any> implements ConfigurationCoordinator<V> { // tslint:disable-line:no-any
    @inject(ConfigurationRunnerDelegator) protected readonly backend: ConfigurationRunnerDelegator;
    @inject(ConfigurationRunnerClient) protected readonly frontend: ConfigurationRunnerClient;

    public readonly id: string;

    protected constructor(@unmanaged() id: string) {
        this.id = id;
    }

    public abstract run(uuid: string, options: V, context: OptionsContext, token: CancellationToken): Promise<void>;
}
