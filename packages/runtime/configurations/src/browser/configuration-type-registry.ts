/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionType } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { ContributionProvider } from "@theia/core";
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { inject, injectable, named } from "inversify";
import { memo } from "helpful-decorators";

export interface ConfigurationType {
    readonly id: string;
    readonly label: string;
    readonly iconClass?: string;
    readonly category?: string;
    readonly options: OptionType[];
}

export const ConfigurationTypeContribution = Symbol("ConfigurationTypeContribution");
export interface ConfigurationTypeContribution {
    registerConfigurationTypes(registry: ConfigurationTypeRegistry): void;
}

export const ConfigurationTypeRegistry = Symbol("ConfigurationTypeRegistry");
export interface ConfigurationTypeRegistry {
    getConfigurationTypes(): ConfigurationType[];
    getConfigurationType(id: string): ConfigurationType | undefined;
    registerConfigurationType(type: ConfigurationType): void;
    unregisterConfigurationType(id: string): void;
}

@injectable()
export class ConfigurationTypeRegistryImpl implements ConfigurationTypeRegistry, FrontendApplicationContribution {
    @inject(ContributionProvider) @named(ConfigurationTypeContribution)
    protected readonly contributions: ContributionProvider<ConfigurationTypeContribution>;

    protected readonly types: Map<String, ConfigurationType>;

    public constructor() {
        this.types = new Map();
    }

    @memo()
    public getConfigurationTypes(): ConfigurationType[] {
        return [...this.types.values()];
    }

    @memo()
    public getConfigurationType(id: string): ConfigurationType | undefined {
        return this.types.get(id);
    }

    public registerConfigurationType(type: ConfigurationType): void {
        const id = type.id;

        if (this.types.has(id)) throw new Error(`There is already a configuration type with id '${id}'.`);
        else this.types.set(id, type);
    }

    public unregisterConfigurationType(id: string): void {
        if (this.types.has(id)) this.types.delete(id);
        else console.warn(`There is no configuration type with id '${id}'.`);
    }

    public onStart(): void {
        this.contributions.getContributions().forEach(contribution => contribution.registerConfigurationTypes(this));
    }
}
