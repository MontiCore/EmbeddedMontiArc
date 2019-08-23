/* (c) https://github.com/MontiCore/monticore */
import { MenuContribution, MenuModelRegistry, MenuPath } from "@theia/core/lib/common";
import { CommonMenus } from "@theia/core/lib/browser";
import { NavigatorContextMenu } from "@theia/navigator/lib/browser/navigator-contribution";
import { WorkspaceCommands } from "@theia/workspace/lib/browser";
import { injectable } from "inversify";

export namespace ModulesMenus {
    export const FILE_NEW = [...CommonMenus.FILE_NEW, "1_new"] as MenuPath;
    export const NAVIGATION_NEW = [...NavigatorContextMenu.NAVIGATION, "1_new"] as MenuPath;
    export const NAVIGATION_NEW_SUBMENU = [...NAVIGATION_NEW, "1_new"] as MenuPath;
}

@injectable()
export class ModulesMenuContribution implements MenuContribution {
    public registerMenus(registry: MenuModelRegistry): void {
        registry.unregisterMenuAction(WorkspaceCommands.NEW_FILE, CommonMenus.FILE_NEW);
        registry.unregisterMenuAction(WorkspaceCommands.NEW_FOLDER, CommonMenus.FILE_NEW);
        registry.unregisterMenuAction(WorkspaceCommands.NEW_FILE, NavigatorContextMenu.NAVIGATION);
        registry.unregisterMenuAction(WorkspaceCommands.NEW_FOLDER, NavigatorContextMenu.NAVIGATION);

        registry.registerSubmenu(CommonMenus.FILE_NEW, "New");
        registry.registerSubmenu(ModulesMenus.NAVIGATION_NEW, "New");

        registry.registerMenuAction(ModulesMenus.FILE_NEW, {
            commandId: WorkspaceCommands.NEW_FILE.id
        });

        registry.registerMenuAction(ModulesMenus.FILE_NEW, {
            commandId: WorkspaceCommands.NEW_FOLDER.id
        });

        registry.registerMenuAction(ModulesMenus.NAVIGATION_NEW_SUBMENU, {
            commandId: WorkspaceCommands.NEW_FILE.id
        });

        registry.registerMenuAction(ModulesMenus.NAVIGATION_NEW_SUBMENU, {
            commandId: WorkspaceCommands.NEW_FOLDER.id
        });
    }
}
