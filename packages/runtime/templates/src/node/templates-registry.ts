/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContributionProvider } from "@theia/core";
import { BackendApplicationContribution } from "@theia/core/lib/node";
import { inject, injectable, named } from "inversify";
import { Template } from "../common";

export const TemplatesContribution = Symbol("TemplatesContribution");
/**
 * Represents a contribution to the templates registry.
 */
export interface TemplatesContribution {
    /**
     * A method which will be called once the templates are registered.
     * @param registry The registry to which the templates will be added.
     */
    registerTemplates(registry: TemplatesRegistry): void;
}

export const TemplatesRegistry = Symbol("TemplatesRegistry");
/**
 * Represents the registry to which the templates will be registered to.
 */
export interface TemplatesRegistry {
    /**
     * Registers a template.
     * @param template The template to be registered.
     */
    registerTemplate(template: Template): void;

    /**
     * Unregisters a template.
     * @param id The id of the template to be unregistered.
     */
    unregisterTemplate(id: string): void;

    /**
     * Fetches all the templates from the registry.
     * @return All the templates registered in the registry.
     */
    getTemplates(): Template[];

    /**
     * Fetches a specific template from the registry.
     * @param id The id of the template to be fetched.
     * @return The template registered under the given id if any, undefined otherwise.
     */
    getTemplate(id: string): Template | undefined;
}

@injectable()
export class TemplatesRegistryImpl implements TemplatesRegistry, BackendApplicationContribution {
    @inject(ContributionProvider) @named(TemplatesContribution)
    protected readonly contributions: ContributionProvider<TemplatesContribution>;

    protected readonly templates: Map<string, Template>;

    public constructor() {
        this.templates = new Map();
    }

    public registerTemplate(template: Template): void {
        const id = template.id;

        if (this.templates.has(id)) throw new Error(`A template with id '${id}' has already been registered.`);
        else this.templates.set(id, template);
    }

    public unregisterTemplate(id: string): void {
        if (this.templates.has(id)) this.templates.delete(id);
        else console.warn(`There is no template with id '${id}'.`);
    }

    public getTemplates(): Template[] {
        return Array.from(this.templates.values());
    }

    public getTemplate(id: string): Template | undefined {
        return this.templates.get(id);
    }

    public onStart(): void {
        this.contributions.getContributions().forEach(contribution => contribution.registerTemplates(this));
    }
}
