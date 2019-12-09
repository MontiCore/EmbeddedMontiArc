/*
 * (c) https://github.com/MontiCore/monticore
 */
import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { ContributionProvider } from "@theia/core/lib/common";
import { memo } from "helpful-decorators";
import { inject, injectable, named } from "inversify";

export interface ModuleType {
    readonly id: string;
    readonly label: string;
    readonly iconClass?: string;
    readonly category?: string;
}

export const ModuleTypeContribution = Symbol("ModuleTypeContribution");
export interface ModuleTypeContribution {
    registerModuleTypes(registry: ModuleTypeRegistry): void;
}

export const ModuleTypeRegistry = Symbol("ModuleTypeRegistry");
export interface ModuleTypeRegistry {
    getModuleTypes(): ModuleType[];
    getModuleType(uuid: string): ModuleType | undefined;
    registerModuleType(type: ModuleType): void;
    unregisterModuleType(id: string): void;
}

@injectable()
export class ModuleTypeRegistryImpl implements ModuleTypeRegistry, FrontendApplicationContribution {
    @inject(ContributionProvider) @named(ModuleTypeContribution)
    protected readonly contributions: ContributionProvider<ModuleTypeContribution>;

    protected readonly types: Map<String, ModuleType>;

    protected constructor() {
        this.types = new Map();
    }

    @memo()
    public getModuleTypes(): ModuleType[] {
        return [...this.types.values()];
    }

    @memo()
    public getModuleType(uuid: string): ModuleType | undefined {
        return this.types.get(uuid);
    }

    public registerModuleType(type: ModuleType): void {
        const id = type.id;

        if (this.types.has(id)) throw new Error(`There is already a module type with id '${id}'.`);
        else this.types.set(id, type);
    }

    public unregisterModuleType(id: string): void {
        if (this.types.has(id)) this.types.delete(id);
        else console.warn(`There is no module type with id '${id}'.`);
    }

    public onStart(): void {
        this.contributions.getContributions().forEach(contribution => contribution.registerModuleTypes(this));
    }
}
