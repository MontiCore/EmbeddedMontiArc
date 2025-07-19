/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Command, CommandContribution, CommandRegistry } from "@theia/core/lib/common";
import { inject, injectable } from "inversify";
import { GUIWindowServer } from "../../common/window";

export namespace GUIWindowCommands {
    export const SHOW: Command = {
        id: "gui.window.show",
        label: "Show Graphical User Interfaces.",
        category: "GUI"
    };
}

@injectable()
export class GUIWindowCommandContribution implements CommandContribution {
    @inject(GUIWindowServer) protected readonly window: GUIWindowServer;

    public registerCommands(registry: CommandRegistry): void {
        registry.registerCommand(GUIWindowCommands.SHOW, {
            isVisible: () => true,
            execute: () => this.window.show()
        });
    }
}
