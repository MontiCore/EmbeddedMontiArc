/*
 * (c) https://github.com/MontiCore/monticore
 */
import { inject, injectable } from "inversify";
import { Template, TemplatesServer } from "../common/templates-protocol";
import * as fs from "fs-extra";
import { TemplatesRegistry } from "./templates-contribution";

@injectable()
export class TemplatesServerImpl implements TemplatesServer {
    @inject(TemplatesRegistry) protected readonly registry: TemplatesRegistry;

    public async getTemplates(): Promise<Template[]> {
        return this.registry.getTemplates();
    }

    public async resolveTemplateContent(id: string): Promise<string> {
        const template = this.registry.getTemplate(id);

        if (template) return fs.readFile(template.path, "UTF-8");
        else return '';
    }
}
