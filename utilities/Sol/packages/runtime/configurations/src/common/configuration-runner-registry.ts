/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContributionProvider } from "@theia/core/lib/common";
import { memo } from "helpful-decorators";
import { inject, injectable, interfaces, named } from "inversify";
import { ContextContainer } from "@embeddedmontiarc/sol-runtime-core/lib/common";
import { ConfigurationRunner } from "./configuration-runner";

import ServiceIdentifier = interfaces.ServiceIdentifier;
import Container = interfaces.Container;

export const ConfigurationRunnerContribution = Symbol("ConfigurationRunnerContribution");
export interface ConfigurationRunnerContribution {
    registerRunners(registry: ConfigurationRunnerRegistry): void;
}

export const ConfigurationRunnerRegistry = Symbol("ConfigurationRunnerRegistry");
export interface ConfigurationRunnerRegistry {
    registerRunner(id: ServiceIdentifier<ConfigurationRunner>): void;
    unregisterRunner(id: string): void;
    getRunner(id: string): ConfigurationRunner | undefined;
    getRunners(): ConfigurationRunner[];
}

@injectable()
export class CommonConfigurationRunnerRegistry implements ConfigurationRunnerRegistry {
    @inject(ContributionProvider) @named(ConfigurationRunnerContribution)
    protected readonly contributions: ContributionProvider<ConfigurationRunnerContribution>;

    @inject(ContextContainer) protected readonly container: Container;

    protected readonly runners: Map<String, ConfigurationRunner>;

    public constructor() {
        this.runners = new Map();
    }

    @memo()
    public getRunners(): ConfigurationRunner[] {
        return [...this.runners.values()];
    }

    @memo()
    public getRunner(id: string): ConfigurationRunner | undefined {
        return this.runners.get(id);
    }

    public registerRunner(id: ServiceIdentifier<ConfigurationRunner>): void {
        const runner = this.container.get(id);

        if (this.runners.has(runner.id)) throw new Error(`There is already a runner with id '${runner.id}'.`);
        else this.runners.set(runner.id, runner);
    }

    public unregisterRunner(id: string): void {
        if (this.runners.has(id)) this.runners.delete(id);
        else console.warn(`There is no runner with id '${id}'.`);
    }
}
