/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ApplicationContribution } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { memo } from "helpful-decorators";
import { inject, injectable, named } from "inversify";
import { ContributionProvider } from "@theia/core/lib/common/contribution-provider";
import { Application } from "express";

/**
 * An interface representing a preparation to be executed during the preparation phase.
 */
export interface Preparation {
    /**
     * The id of the preparation to be executed.
     */
    readonly id: string;

    /**
     * The priority of the preparation amongst its peers. Higher means sooner.
     */
    readonly priority: number;

    /**
     * A method which is performing the actual preparation.
     */
    readonly prepare: () => Promise<void>;
}

export const PreparationContribution = Symbol("PreparationContribution");
/**
 * An interface to be implemented by classes which register preparations.
 */
export interface PreparationContribution {
    /**
     * A method which will be called once the preparations are getting registered.
     * @param registry The registry to which the preparations are added.
     */
    registerPreparations(registry: PreparationRegistry): void;
}

export const PreparationRegistry = Symbol("PreparationRegistry");
/**
 * An interface to be implemented by classes which implement the necessary functionality to fetch, register, and
 * unregister preparations.
 */
export interface PreparationRegistry {
    /**
     * Fetches all registered preparations.
     * @return All registered preparations.
     */
    getPreparations(): Preparation[];

    /**
     * Fetches all registered preparations sorted via their priority.
     * @return A sorted array with all registered preparations.
     */
    getSortedPreparations(): Preparation[];

    /**
     * Registers a given preparations.
     * @param preparation The preparation to be registered.
     */
    registerPreparation(preparation: Preparation): void;

    /**
     * Unregisters the preparation with the given id.
     * @param id The id of the preparation to be unregistered.
     */
    unregisterPreparation(id: string): void;
}

@injectable()
export class PreparationRegistryImpl implements PreparationRegistry, ApplicationContribution {
    @inject(ContributionProvider) @named(PreparationContribution)
    protected readonly provider: ContributionProvider<PreparationContribution>;

    protected readonly preparations: Map<string, Preparation>;

    public constructor() {
        this.preparations = new Map();
    }

    @memo()
    public getPreparations(): Preparation[] {
        return Array.from(this.preparations.values());
    }

    @memo()
    public getSortedPreparations(): Preparation[] {
        return this.getPreparations().sort((a, b) => b.priority - a.priority);
    }

    public registerPreparation(preparation: Preparation): void {
        const id = preparation.id;

        if (this.preparations.has(id)) throw new Error(`${id} is already a preparation.`);
        else this.preparations.set(preparation.id, preparation);
    }

    public unregisterPreparation(id: string): void {
        if (this.preparations.has(id)) this.preparations.delete(id);
        else throw new Error(`${id} is not a valid preparation.`);
    }

    public async onConfigure(application: Application): Promise<void> {
        this.provider.getContributions().forEach(preparation => preparation.registerPreparations(this));
    }
}
