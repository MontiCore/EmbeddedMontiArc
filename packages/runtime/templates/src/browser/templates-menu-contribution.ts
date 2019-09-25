/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ModulesMenus } from "@embeddedmontiarc/sol-runtime-modules/lib/browser/modules-menu-contribution";
import { MenuContribution, MenuModelRegistry, MenuPath } from "@theia/core";
import { inject, injectable } from "inversify";
import { Template, TemplatesServer } from "../common/templates-protocol";
import { TemplatesCommands } from "./templates-command-contribution";

export namespace TemplatesMenus {
    export const FILE_NEW = [...ModulesMenus.FILE_NEW, "2_new_from_template"] as MenuPath;
    export const NAVIGATION_NEW = [...ModulesMenus.NAVIGATION_NEW, "2_new_from_template"] as MenuPath;
}

@injectable()
export class TemplatesMenuContribution implements MenuContribution {
    @inject(TemplatesServer) protected readonly server: TemplatesServer;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        const templates = await this.server.getTemplates();

        templates.forEach((template: Template) => {
            const command = TemplatesCommands.NEW_FILE_FROM_TEMPLATE(template.id);
            const commandId = command.id;
            const label = `New ${template.label}`;
            const menuAction = { commandId, label };

            registry.registerMenuAction(TemplatesMenus.FILE_NEW, menuAction);
            registry.registerMenuAction(TemplatesMenus.NAVIGATION_NEW, menuAction);
        });
    }
}
