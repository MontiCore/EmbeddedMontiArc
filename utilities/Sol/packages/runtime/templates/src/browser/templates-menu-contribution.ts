/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CoreMenus } from "@embeddedmontiarc/sol-runtime-core/lib/browser/core-menu-contribution";
import { MenuAction, MenuContribution, MenuModelRegistry, MenuPath } from "@theia/core/lib/common";
import { inject, injectable } from "inversify";
import { Template, TemplatesServer } from "../common";
import { TemplatesCommands } from "./templates-command-contribution";

export namespace TemplatesMenus {
    export const FILE_NEW = [...CoreMenus.FILE_NEW, "3_new_template"] as MenuPath;
    export const NAVIGATION_NEW = [...CoreMenus.NAVIGATION_NEW, "3_new_template"] as MenuPath;
}

@injectable()
export class TemplatesMenuContribution implements MenuContribution {
    @inject(TemplatesServer) protected readonly server: TemplatesServer;

    public async registerMenus(registry: MenuModelRegistry): Promise<void> {
        const templates = await this.server.getTemplates();

        templates.forEach((template: Template) => {
            const command = TemplatesCommands.NEW_FILE_FROM_TEMPLATE(template.id);
            const commandId = command.id;
            const label = `New ${template.label}...`;
            const menuAction = { commandId, label } as MenuAction;

            registry.registerMenuAction(TemplatesMenus.FILE_NEW, menuAction);
            registry.registerMenuAction(TemplatesMenus.NAVIGATION_NEW, menuAction);
        });
    }
}
