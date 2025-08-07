/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Application, ApplicationPhase } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { WindowService } from "@embeddedmontiarc/sol-external-core/lib/main/window/window-service";
import { inject, injectable } from "inversify";

@injectable()
export class WorkspacePhase implements ApplicationPhase {
    @inject(Application) protected readonly application: Application;
    @inject(WindowService) protected readonly windows: WindowService;

    public get id(): string {
        return "workspace";
    }

    public get priority(): number {
        return 40000;
    }

    public async prepare(): Promise<void> {
        const port = this.application.getPort();
        const window = this.windows.switch({
            show: false,
            title: this.application.getName(),
            resizable: false,
            webPreferences: { nodeIntegration: true },
            height: 560,
            width: 1100
        });

        window.setMenuBarVisibility(false);
        // window.webContents.openDevTools();
        return window.loadURL(`http://localhost:${port}/#/workspaces`);
    }
}
