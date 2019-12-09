/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionManager } from "@embeddedmontiarc/sol-runtime-options/lib/browser/option-manager";
import { ValidatorServer } from "@embeddedmontiarc/sol-runtime-options/lib/common";
import { open, SingleTextInputDialog } from "@theia/core/lib/browser";
import { Command, CommandContribution, CommandRegistry } from "@theia/core/lib/common";
import { UriCommandHandler } from "@theia/core/lib/common/uri-command-handler";
import { FileStat, FileSystemUtils } from "@theia/filesystem/lib/common";
import { WorkspaceCommandContribution } from "@theia/workspace/lib/browser";
import { inject, injectable } from "inversify";
import { Template, TemplatesServer } from "../common";
import { TemplateDialogFactory } from "./template-dialog";

import URI from "@theia/core/lib/common/uri";

export namespace TemplatesCommands {
    export const NEW_FILE_FROM_TEMPLATE = (id: string): Command => ({
        id: `file.newFileFromTemplate.${id}`
    });
}

@injectable()
export class TemplatesCommandContribution extends WorkspaceCommandContribution implements CommandContribution {
    @inject(OptionManager) protected readonly manager: OptionManager;
    @inject(TemplatesServer) protected readonly server: TemplatesServer;
    @inject(ValidatorServer) protected readonly validator: ValidatorServer;
    @inject(TemplateDialogFactory) protected readonly dialogFactory: TemplateDialogFactory;

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
                const options = await this.showTemplateDialog(template);

                if (options) {
                    const childURI = parentURI.resolve(name);

                    await this.server.renderTo(template.id, childURI.toString(), options);
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

    protected async showTemplateDialog(template: Template): Promise<object | undefined> {
        const dialog = this.dialogFactory({
            id: template.id,
            title: `New ${template.label}`,
            options: template.options
        });

        return dialog.open();
    }
}
