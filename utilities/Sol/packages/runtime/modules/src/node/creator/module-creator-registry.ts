/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContextContainer } from "@embeddedmontiarc/sol-runtime-core/lib/common";
import { ContributionProvider } from "@theia/core";
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { memo } from "helpful-decorators";
import { inject, injectable, interfaces, named } from "inversify";
import { ModuleCreator } from "./module-creator";

import Container = interfaces.Container;
import ServiceIdentifier = interfaces.ServiceIdentifier;

export const ModuleCreatorContribution = Symbol("ModuleCreatorContribution");
export interface ModuleCreatorContribution {
    registerModuleCreators(registry: ModuleCreatorRegistry): void;
}

export const ModuleCreatorRegistry = Symbol("ModuleCreatorRegistry");
export interface ModuleCreatorRegistry {
    registerModuleCreator(id: ServiceIdentifier<ModuleCreator>): void;
    unregisterModuleCreator(id: string): void;
    getModuleCreators(): ModuleCreator[];
    getModuleCreator(id: string): ModuleCreator | undefined;
}

@injectable()
export class ModuleCreatorRegistryImpl implements ModuleCreatorRegistry, BackendApplicationContribution {
    @inject(ContributionProvider) @named(ModuleCreatorContribution)
    protected readonly contributions: ContributionProvider<ModuleCreatorContribution>;

    @inject(ContextContainer) protected readonly container: Container;

    protected readonly creators: Map<String, ModuleCreator>;

    public constructor() {
        this.creators = new Map();
    }

    @memo()
    public getModuleCreators(): ModuleCreator[] {
        return [...this.creators.values()];
    }

    public getModuleCreator(id: string): ModuleCreator | undefined {
        return this.creators.get(id);
    }

    public registerModuleCreator(id: ServiceIdentifier<ModuleCreator>): void {
        const creator = this.container.get(id);

        if (this.creators.has(creator.id)) throw new Error(`There is already a creator with id '${creator.id}'.`);
        else this.creators.set(creator.id, creator);
    }

    public unregisterModuleCreator(id: string): void {
        if (this.creators.has(id)) this.creators.delete(id);
        else throw new Error(`There is no creator with id '${id}'.`);
    }

    public onStart(): void {
        this.contributions.getContributions().forEach(contribution => contribution.registerModuleCreators(this));
    }
}
