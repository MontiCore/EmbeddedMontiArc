/*
 * (c) https://github.com/MontiCore/monticore
 */
import { FileUri } from "@theia/core/lib/node";
import { inject, injectable } from "inversify";
import { Template, TemplatesServer } from "../common";
import { TemplatesRegistry } from "./templates-registry";

import * as path from "path";
import * as nunjucks from "nunjucks";
import * as fs from "fs-extra";
import * as util from "util";

@injectable()
export class TemplatesServerImpl implements TemplatesServer {
    @inject(TemplatesRegistry) protected readonly registry: TemplatesRegistry;

    public async getTemplates(): Promise<Template[]> {
        return this.registry.getTemplates();
    }

    public async renderTo(id: string, destination: string, options: object): Promise<void> {
        const template = this.registry.getTemplate(id);

        if (template) return this.doRenderTo(template, destination, options);
        else throw new Error(`Could not locate template with ID ${id}.`);
    }

    protected async doRenderTo(template: Template, uri: string, options: object): Promise<void> {
        const dirname = path.dirname(template.path);
        const basename = path.basename(template.path);

        nunjucks.configure(dirname, { autoescape: false, trimBlocks: true });

        const content = await this.render(basename, options);
        const destination = FileUri.fsPath(uri);

        return fs.writeFile(destination, content);
    }

    protected async render(basename: string, context: object): Promise<string> {
        const render = util.promisify<string, object, string>(nunjucks.render);

        return render(basename, context);
    }
}
