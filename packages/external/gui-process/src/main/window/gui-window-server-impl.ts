/*
 * (c) https://github.com/MontiCore/monticore
 */
import { WindowService } from "@embeddedmontiarc/sol-external-core/lib/main/window/window-service";
import { DockerNetworkService } from "@embeddedmontiarc/sol-external-docker/lib/main";
import { WorkspaceServer } from "@embeddedmontiarc/sol-external-workspace/lib/common";
import { bind } from "helpful-decorators";
import { inject, injectable, postConstruct } from "inversify";
import { GUIWindowServer } from "../../common";
import { BrowserWindow } from "electron";

// TODO: Make dependent on active theme.
const CSS: string = `
    body {
        background:#1d1d1d !important;
    }

    #progress > * {
        opacity:0 !important;
    }

    #top_bar {
        display:none !important;
    }
`;

@injectable()
export class GUIWindowServerImpl implements GUIWindowServer {
    @inject(WindowService) protected readonly windows: WindowService;
    @inject(DockerNetworkService) protected readonly network: DockerNetworkService;
    @inject(WorkspaceServer) protected readonly workspaces: WorkspaceServer;

    @postConstruct()
    protected init(): void {
        this.workspaces.on("close", this.onWorkspaceClose);
    }

    protected window: BrowserWindow | undefined;

    public async show(): Promise<void> {
        if (this.window) this.window.focus();
        else this.create();
    }

    protected async create(): Promise<void> {
        this.window = this.windows.create({
            title: "Graphical User Interfaces",
            show: false
        });

        if (this.window) return this.prepareWindow(this.window);
    }

    protected async prepareWindow(window: BrowserWindow): Promise<void> {
        const internalIP = this.network.getInternalIP();
        const port = this.network.resolve(10000);

        window.once("close", () => this.window = undefined);
        window.removeAllListeners("ready-to-show");
        window.setMenuBarVisibility(false);
        await window.loadURL(`http://${internalIP}:${port}?top_bar=false`);
        window.webContents.insertCSS(CSS);
        window.show();
        window.maximize();
    }

    @bind
    protected onWorkspaceClose(): void {
        if (this.window) this.window.close();
    }
}
