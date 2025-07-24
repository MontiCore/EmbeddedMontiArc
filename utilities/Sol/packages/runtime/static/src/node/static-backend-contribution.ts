/*
 * (c) https://github.com/MontiCore/monticore
 */
import { BackendApplicationContribution, FileUri } from "@theia/core/lib/node";
import { WorkspaceServer } from "@theia/workspace/lib/common";
import { inject, injectable } from "inversify";
import { Application, static as serve } from "express";

import URI from "@theia/core/lib/common/uri";

@injectable()
export class StaticBackendContribution implements BackendApplicationContribution {
    @inject(WorkspaceServer) protected readonly workspaceServer: WorkspaceServer;

    public configure(application: Application): void {
        application.use("/workspace/:path", async (request, response, next) => {
            const workspace = await this.workspaceServer.getMostRecentlyUsedWorkspace(); // TODO: Fetch all workspace roots from frontend.
            const workspaceUri = workspace && new URI(workspace);
            const uri = workspaceUri && workspaceUri.resolve(decodeURIComponent(request.params.path));

            if (uri) return serve(FileUri.fsPath(uri))(request, response, next);
            else next();
        });
    }
}
