/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable, inject } from "inversify";
import { FrontendApplication as FrontendApplicationBase } from "@theia/core/lib/browser";
import { WorkspaceServer } from "../common";

@injectable()
export class FrontendApplication extends FrontendApplicationBase {
    @inject(WorkspaceServer) protected readonly server: WorkspaceServer;

    protected registerEventListeners(): void {
        super.registerEventListeners();

        this.server.on("close", () => {
            this.stateService.state = "closing_window";
            this.layoutRestorer.storeLayout(this);
            this.stopContributions();
        });
    }
}
