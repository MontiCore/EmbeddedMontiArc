/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContextContainer } from "@embeddedmontiarc/sol-runtime-core/lib/common";
import { ContributionProvider } from "@theia/core";
import { memo } from "helpful-decorators";
import { inject, injectable, interfaces, named } from "inversify";
import { ConfigurationCoordinator } from "./configuration-coordinator";

import Container = interfaces.Container;
import ServiceIdentifier = interfaces.ServiceIdentifier;

export const ConfigurationCoordinatorContribution = Symbol("ConfigurationCoordinatorContribution");
export interface ConfigurationCoordinatorContribution {
    registerCoordinators(registry: ConfigurationCoordinatorRegistry): void;
}

export const ConfigurationCoordinatorRegistry = Symbol("ConfigurationCoordinatorRegistry");
export interface ConfigurationCoordinatorRegistry {
    registerCoordinator(id: ServiceIdentifier<ConfigurationCoordinator>): void;
    unregisterCoordinator(id: string): void;
    getCoordinators(): ConfigurationCoordinator[];
    getCoordinator(id: string): ConfigurationCoordinator | undefined;
}

@injectable()
export class ConfigurationCoordinatorRegistryImpl implements ConfigurationCoordinatorRegistry {
    @inject(ContributionProvider) @named(ConfigurationCoordinatorContribution)
    protected readonly contributions: ContributionProvider<ConfigurationCoordinatorContribution>;

    @inject(ContextContainer) protected readonly container: Container;

    protected readonly coordinators: Map<String, ConfigurationCoordinator>;

    public constructor() {
        this.coordinators = new Map();
    }

    @memo()
    public getCoordinators(): ConfigurationCoordinator[] {
        return [...this.coordinators.values()];
    }

    @memo()
    public getCoordinator(id: string): ConfigurationCoordinator | undefined {
        return this.coordinators.get(id);
    }

    public registerCoordinator(id: ServiceIdentifier<ConfigurationCoordinator>): void {
        const coordinator = this.container.get(id);

        if (this.coordinators.has(coordinator.id)) throw new Error(`There is already a coordinator with id '${coordinator.id}'.`);
        else this.coordinators.set(coordinator.id, coordinator);
    }

    public unregisterCoordinator(id: string): void {
        if (this.coordinators.has(id)) this.coordinators.delete(id);
        else console.warn(`There is no coordinator with id '${id}'.`);
    }

    public onStart(): void {
        this.contributions.getContributions().forEach(contribution => contribution.registerCoordinators(this));
    }
}
