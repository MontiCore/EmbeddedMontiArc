/*
 * (c) https://github.com/MontiCore/monticore
 */
import { MenuPath, MenuContribution, MenuModelRegistry } from "@theia/core/lib/common";
import { CommonMenus } from "@theia/core/lib/browser";
import { NavigatorContextMenu } from "@theia/navigator/lib/browser/navigator-contribution";
import { WorkspaceCommands } from "@theia/workspace/lib/browser";
import { injectable } from "inversify";

export namespace CoreMenus {
    export const FILE_NEW: MenuPath = [...CommonMenus.FILE_NEW, "1_new"];
    export const FILE_NEW_SUBMENU: MenuPath = [...FILE_NEW, "1_new"];
    export const NAVIGATION_NEW: MenuPath = [...NavigatorContextMenu.NAVIGATION, "1_new"];
    export const NAVIGATION_NEW_SUBMENU: MenuPath = [...NAVIGATION_NEW, "1_new"];
}

@injectable()
export class CoreMenuContribution implements MenuContribution {
    public registerMenus(registry: MenuModelRegistry): void {
        registry.unregisterMenuAction(WorkspaceCommands.NEW_FILE, CommonMenus.FILE_NEW);
        registry.unregisterMenuAction(WorkspaceCommands.NEW_FOLDER, CommonMenus.FILE_NEW);
        registry.unregisterMenuAction(WorkspaceCommands.NEW_FILE, NavigatorContextMenu.NAVIGATION);
        registry.unregisterMenuAction(WorkspaceCommands.NEW_FOLDER, NavigatorContextMenu.NAVIGATION);

        registry.registerSubmenu(CommonMenus.FILE_NEW, "New");
        registry.registerSubmenu(CoreMenus.NAVIGATION_NEW, "New");

        registry.registerMenuAction(CoreMenus.FILE_NEW_SUBMENU, {
            commandId: WorkspaceCommands.NEW_FILE.id
        });

        registry.registerMenuAction(CoreMenus.FILE_NEW_SUBMENU, {
            commandId: WorkspaceCommands.NEW_FOLDER.id
        });

        registry.registerMenuAction(CoreMenus.NAVIGATION_NEW_SUBMENU, {
            commandId: WorkspaceCommands.NEW_FILE.id
        });

        registry.registerMenuAction(CoreMenus.NAVIGATION_NEW_SUBMENU, {
            commandId: WorkspaceCommands.NEW_FOLDER.id
        });
    }
}
