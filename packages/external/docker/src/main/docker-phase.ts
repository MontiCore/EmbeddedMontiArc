/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Application, ApplicationPhase } from "@embeddedmontiarc/sol-external-core/lib/main/application";
import { WindowService } from "@embeddedmontiarc/sol-external-core/lib/main/window/window-service";
import { bind } from "helpful-decorators";
import { inject, injectable } from "inversify";
import { DockerRunService } from "./docker-run-service";
import { DockerNetworkService } from "./docker-network-service";
import { DockerService } from "./docker-service";
import { Event, BrowserWindow } from "electron";

@injectable()
export class DockerPhase implements ApplicationPhase {
    @inject(Application) protected readonly application: Application;
    @inject(DockerService) protected readonly service: DockerService;
    @inject(DockerRunService) protected readonly runner: DockerRunService;
    @inject(WindowService) protected readonly windows: WindowService;
    @inject(DockerNetworkService) protected readonly network: DockerNetworkService;

    public get id(): string {
        return "docker";
    }

    public get priority(): number {
        return 30000;
    }

    public async prepare(hostPath: string): Promise<void> {
        await this.runner.run(hostPath);

        const internalIP = this.network.getInternalIP();
        const externalPort = this.network.resolve(3000);
        const window = this.windows.switch({
            title: this.application.getName(),
            webPreferences: { nodeIntegration: true },
            show: false
        });

        window.setMenuBarVisibility(false);
        // window.webContents.openDevTools();
        window.once("close", event => this.onWindowClose(event, window));
        await window.loadURL(`http://${internalIP}:${externalPort}`);
        window.maximize();
    }

    @bind
    protected async onWindowClose(event: Event, window: BrowserWindow): Promise<void> {
        event.preventDefault();
        await window.webContents.executeJavaScript(`window.dispatchEvent(new Event("unload"));`);
        window.close();
    }
}
