/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { CancellationToken } from "@theia/core/lib/common";
import { injectable, unmanaged } from "inversify";

export interface ConfigurationRunner<V = any> { // tslint:disable-line:no-any
    readonly id: string;

    run(uuid: string, taskName: string, options: V, context: OptionsContext, token: CancellationToken): Promise<void>;
}

@injectable()
export abstract class CommonConfigurationRunner<V = any> implements ConfigurationRunner<V> { // tslint:disable-line:no-any
    public readonly id: string;

    protected constructor(@unmanaged() id: string) {
        this.id = id;
    }

    public abstract run(uuid: string, taskName: string, options: V, context: OptionsContext, token: CancellationToken): Promise<void>;
}
