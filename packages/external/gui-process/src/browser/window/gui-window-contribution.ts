/*
 * (c) https://github.com/MontiCore/monticore
 */
import { FrontendApplication, FrontendApplicationContribution, StatusBar, StatusBarAlignment } from "@theia/core/lib/browser";
import { inject, injectable } from "inversify";
import { GUIWindowCommands } from "./gui-window-commands";

@injectable()
export class GUIWindowContribution implements FrontendApplicationContribution {
    @inject(StatusBar) protected readonly statusBar: StatusBar;

    public static readonly id: string = "gui-processes";

    public onStart(application: FrontendApplication): void {
        this.statusBar.setElement(GUIWindowContribution.id, {
            text: "$(desktop)",
            alignment: StatusBarAlignment.RIGHT,
            priority: -1100,
            command: GUIWindowCommands.SHOW.id,
            tooltip: GUIWindowCommands.SHOW.label
        }).catch(error => console.error(error.message));
    }
}
