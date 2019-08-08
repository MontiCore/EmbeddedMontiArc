/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */

// tslint:disable:no-any

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
     * The elements which will be passed to the dynamic dialog.
     */
    readonly elements: any;

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
     * Resolves the content of the template registered under the given id.
     * @param id The id of the template whose content should be resolved.
     */
    resolveTemplateContent(id: string): Promise<string>;
}
