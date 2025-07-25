/*
 * (c) https://github.com/MontiCore/monticore
 */
import { CoreMenus } from "@embeddedmontiarc/sol-runtime-core/lib/browser";
import { MenuContribution, MenuModelRegistry, MenuPath } from "@theia/core/lib/common";
import { injectable } from "inversify";
import { ModuleCommands } from "./module-command-contribution";

export namespace ModulesMenus {
    export const FILE_NEW = [...CoreMenus.FILE_NEW, "2_new_module"] as MenuPath;
    export const NAVIGATION_NEW = [...CoreMenus.NAVIGATION_NEW, "2_new_module"] as MenuPath;
}

@injectable()
export class ModuleMenuContribution implements MenuContribution {
    public registerMenus(registry: MenuModelRegistry): void {
        registry.registerMenuAction(ModulesMenus.FILE_NEW, {
            label: "Module...",
            commandId: ModuleCommands.NEW_MODULE.id,
            icon: "fa fa-th"
        });

        registry.registerMenuAction(ModulesMenus.NAVIGATION_NEW, {
            label: "Module...",
            commandId: ModuleCommands.NEW_MODULE.id,
            icon: "fa fa-th"
        });
    }
}
