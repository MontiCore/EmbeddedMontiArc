/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ApplicationServer } from "@embeddedmontiarc/sol-external-core/lib/common";
import { Application, ApplicationPhase } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { NotificationsService } from "@embeddedmontiarc/sol-external-core/lib/main/messages";
import { WindowService } from "@embeddedmontiarc/sol-external-core/lib/main/window/window-service";
import { MessageType } from "@theia/core";
import { delay } from "helpful-decorators";
import { inject, injectable } from "inversify";
import { PreparationRegistry } from "./preparation-registry";

@injectable()
export class PreparationPhase implements ApplicationPhase {
    @inject(WindowService) protected readonly windows: WindowService;
    @inject(NotificationsService) protected readonly notifications: NotificationsService;
    @inject(PreparationRegistry) protected readonly registry: PreparationRegistry;
    @inject(Application) protected readonly application: Application;
    @inject(ApplicationServer) protected readonly server: ApplicationServer;

    public get id(): string {
        return "preparation";
    }

    public get priority(): number {
        return 50000;
    }

    public async prepare(): Promise<void> {
        const port = this.application.getPort() || 0;
        const window = this.windows.switch({
            title: this.application.getName(),
            frame: false,
            show: false,
            webPreferences: { nodeIntegration: true },
            resizable: false,
            height: 331,
            width: 1100
        });

        // window.webContents.openDevTools();
        window.setMenuBarVisibility(false);
        return window.loadURL(`http://localhost:${port}`);
    }

    @delay(500) // For purely cosmetic purposes.
    public async execute(): Promise<void> {
        try {
            await this.doPrepare();
        } catch (error) {
            await this.notifications.reportProgress(1, 1);
            await this.notifications.showMessage(error.message, MessageType.Error);
        }
    }

    protected async doPrepare(): Promise<void> {
        const preparations = this.registry.getSortedPreparations();
        const total = preparations.length;

        for (let i = 0; i < total; i++) {
            await preparations[i].prepare();
            await this.notifications.reportProgress(i + 1, total);
        }

        await this.notifications.showMessage("Launching application...");
        return this.server.nextPhase();
    }
}
