/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionsContext } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { Disposable, DisposableCollection } from "@theia/core/lib/common";
import { inject, injectable, postConstruct } from "inversify";
import { ConfigurationProcessor } from "./configuration-processor";

import {
    Configuration, ConfigurationExitEvent,
    ConfigurationOutputEvent,
    ConfigurationProcessorClient,
    ConfigurationProcessorServer, ConfigurationStartEvent
} from "../../common";

@injectable()
export class ConfigurationProcessorServerImpl implements ConfigurationProcessorServer, Disposable {
    @inject(ConfigurationProcessor) protected readonly processor: ConfigurationProcessor;

    protected readonly clients: ConfigurationProcessorClient[];
    protected readonly toDispose: DisposableCollection;

    public constructor() {
        this.clients = [];
        this.toDispose = new DisposableCollection();
    }

    @postConstruct()
    protected init(): void {
        this.toDispose.pushAll([
            this.processor.onConfigurationExit(event => this.fireConfigurationExit(event)),
            this.processor.onConfigurationStart(event => this.fireConfigurationStart(event)),
            this.processor.onConfigurationOutput(event => this.fireConfigurationOutput(event))
        ]);
    }

    public async isConfigurationRunning(uuid: string): Promise<boolean> {
        return this.processor.isRunning(uuid);
    }

    public async getRunningConfigurations(): Promise<string[]> {
        return this.processor.getRunningConfigurations();
    }

    public async killConfiguration(uuid: string): Promise<void> {
        return this.processor.kill(uuid);
    }

    public async runConfiguration(configuration: Configuration, context: OptionsContext): Promise<void> {
        return this.processor.run(configuration, context);
    }

    protected fireConfigurationStart(event: ConfigurationStartEvent): void {
        this.clients.forEach(client => client.onConfigurationStart(event));
    }

    protected fireConfigurationExit(event: ConfigurationExitEvent): void {
        this.clients.forEach(client => client.onConfigurationExit(event));
    }

    protected fireConfigurationOutput(event: ConfigurationOutputEvent): void {
        this.clients.forEach(client => client.onConfigurationOutput(event));
    }

    public dispose(): void {
        this.toDispose.dispose();
    }

    public setClient(client: ConfigurationProcessorClient): void {
        this.clients.push(client);
    }

    public unsetClient(client: ConfigurationProcessorClient): void {
        const index = this.clients.indexOf(client);

        if (index > -1) this.clients.splice(index, 1);
    }
}
