/*
 * (c) https://github.com/MontiCore/monticore
 */

import { OptionType } from "@embeddedmontiarc/sol-runtime-options/lib/common";

export const TEMPLATE_VALIDATOR_TYPE: string = "template";

/**
 * Represents a registered template.
 */
export interface Template {
    /**
     * The identifier under which the template will be registered.
     */
    readonly id: string;

    /**
     * The label paired with "New" used in the new file menu.
     */
    readonly label: string;

    /**
     * The options which will be passed to the dynamic dialog.
     */
    readonly options: OptionType[];

    /**
     * The storage location of the template.
     */
    readonly path: string;

    /**
     * The file extension of the resulting file without leading dot.
     */
    readonly extension: string;
}

export namespace TemplatesPaths {
    export const PATH: string = "/services/templates";
}

export const TemplatesServer = Symbol("TemplatesServer");
/**
 * Represents a server for templates.
 */
export interface TemplatesServer {
    /**
     * Fetches all the templates from the registry.
     * @return All the templates which have been registered.
     */
    getTemplates(): Promise<Template[]>;

    /**
     * Renders an instance of the template registered under the given id to a given destination.
     * @param id The id of the template whose content should be resolved.
     * @param destination The destination URI to which a instance of the template should be rendered to.
     * @param options The options to be passed as context to the template engine.
     */
    renderTo(id: string, destination: string, options: object): Promise<void>;
}
