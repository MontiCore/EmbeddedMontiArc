/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { CancellationTokenSource } from "@theia/core/lib/common";
import { inject, injectable } from "inversify";
import { ConfigurationRunnerClient } from "../../common";
import { ConfigurationRunnerDelegator } from "../../common";

@injectable()
export class ConfigurationRunnerClientImpl implements ConfigurationRunnerClient, FrontendApplicationContribution {
    @inject(ConfigurationRunnerDelegator) protected readonly delegator: ConfigurationRunnerDelegator;

    protected readonly tokens: Map<string, CancellationTokenSource>;

    public constructor() {
        this.tokens = new Map();
    }

    public async run<V>(uuid: string, typeId: string, taskName: string, options: V, context: OptionsContext): Promise<void> {
        const source = new CancellationTokenSource();

        this.tokens.set(uuid, source);
        return this.delegator.run(uuid, typeId, taskName, options, context, source.token);
    }

    public async kill(uuid: string): Promise<void> {
        const token = this.tokens.get(uuid);

        if (token) {
            token.cancel();
            this.tokens.delete(uuid);
        }
    }

    public async dispose(uuid: string): Promise<void> {
        const token = this.tokens.get(uuid);

        if (token) {
            token.dispose();
            this.tokens.delete(uuid);
        }
    }

    public onStart() { /* noop */ }
}
