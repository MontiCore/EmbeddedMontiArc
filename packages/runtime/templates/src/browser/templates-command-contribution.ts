/* (c) https://github.com/MontiCore/monticore */
import { DynamicDialogProps } from "@embeddedmontiarc/sol-runtime-components/lib/browser";
import { ComponentManager } from "@embeddedmontiarc/sol-runtime-components/lib/browser/component-manager";
import { open, SingleTextInputDialog } from "@theia/core/lib/browser";
import { Command, CommandContribution, CommandRegistry } from "@theia/core/lib/common";
import URI from "@theia/core/lib/common/uri";
import { UriCommandHandler } from "@theia/core/lib/common/uri-command-handler";
import { FileStat, FileSystemUtils } from "@theia/filesystem/lib/common";
import { WorkspaceCommandContribution } from "@theia/workspace/lib/browser";
import { inject, injectable } from "inversify";
import { Template, TemplatesServer } from "../common/templates-protocol";
import * as nunjucks from "nunjucks/browser/nunjucks";
import { TemplateVariablesDialog } from "./template-variables-dialog";

export namespace TemplatesCommands {
    export const NEW_FILE_FROM_TEMPLATE = (id: string): Command => ({
        id: `file.newFileFromTemplate.${id}`
    });
}

@injectable()
export class TemplatesCommandContribution extends WorkspaceCommandContribution implements CommandContribution {
    @inject(ComponentManager) protected readonly manager: ComponentManager;
    @inject(TemplatesServer) protected readonly server: TemplatesServer;

    public async registerCommands(registry: CommandRegistry): Promise<void> {
        const templates = await this.server.getTemplates();

        templates.forEach((template: Template) => {
            const command = TemplatesCommands.NEW_FILE_FROM_TEMPLATE(template.id);
            const handler = this.newWorkspaceRootUriAwareCommandHandler(this.getNewFileFromTemplateHandler(template));

            registry.registerCommand(command, handler);
        });
    }

    protected getNewFileFromTemplateHandler(template: Template): UriCommandHandler<URI> {
        return {
            execute: (uri: URI) => this.execute(uri, template)
        };
    }

    protected async execute(uri: URI, template: Template): Promise<void> {
        const parent = await this.getDirectory(uri);

        if (parent) {
            const parentURI = new URI(parent.uri);
            const name = await this.showNewFileDialog(template, parent);

            if (name) {
                const context = await this.showTemplateVariablesDialog(template);

                if (context) {
                    const childURI = parentURI.resolve(name);
                    const content = await this.renderTemplate(template, context);

                    await this.fileSystem.createFile(childURI.toString(), { content });
                    await open(this.openerService, childURI);
                }
            }
        }
    }

    protected async showNewFileDialog(template: Template, parent: FileStat): Promise<string | undefined> {
        const config = this.getDefaultFileConfig();
        const parentURI = new URI(parent.uri);
        const childURI = FileSystemUtils.generateUniqueResourceURI(parentURI, parent, config.fileName, template.extension);

        const dialog = new SingleTextInputDialog({
            title: `New ${template.label}`,
            initialValue: childURI.path.base,
            validate: name => this.validateFileName(name, parent, true)
        });

        return dialog.open();
    }

    protected async showTemplateVariablesDialog(template: Template): Promise<object | undefined> {
        const props = { title: `New ${template.label}`, elements: template.elements } as DynamicDialogProps;
        const dialog = new TemplateVariablesDialog(this.manager, props);

        return dialog.open();
    }

    protected async renderTemplate(template: Template, context: object): Promise<string> {
        const content = await this.server.resolveTemplateContent(template.id);

        return this.renderString(content, context);
    }

    protected async renderString(content: string, context: object): Promise<string> {
        return new Promise((resolve, reject) => {
            nunjucks.configure({ autoescape: false, trimBlocks: true });
            nunjucks.renderString(content, context, (error: Error | string | undefined, result: string) => {
                if (error) reject(error);
                else resolve(result);
            });
        });
    }
}
