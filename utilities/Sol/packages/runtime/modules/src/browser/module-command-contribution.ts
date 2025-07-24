/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ConfigurationManager } from "@embeddedmontiarc/sol-runtime-configurations/lib/browser/configuration-manager";
import { Command, CommandContribution, CommandRegistry } from "@theia/core/lib/common";
import { WorkspaceCommandContribution, WorkspaceService } from "@theia/workspace/lib/browser";
import { inject, injectable } from "inversify";
import { ModuleServer } from "../common";
import { ModuleDialogFactory } from "./module-dialog";

import URI from "@theia/core/lib/common/uri";

export namespace ModuleCommands {
    export const NEW_MODULE: Command = {
        id: "module-dialog.show"
    };
}

@injectable()
export class ModuleCommandContribution extends WorkspaceCommandContribution implements CommandContribution {
    @inject(ModuleDialogFactory) protected readonly factory: ModuleDialogFactory;
    @inject(ModuleServer) protected readonly server: ModuleServer;
    @inject(ConfigurationManager) protected readonly manager: ConfigurationManager;
    @inject(WorkspaceService) protected readonly workspace: WorkspaceService;

    public registerCommands(registry: CommandRegistry): void {
        const handler = { execute: (uri: URI) => this.showDialog(uri) };
        const uriHandler = this.newWorkspaceRootUriAwareCommandHandler(handler);

        registry.registerCommand(ModuleCommands.NEW_MODULE, uriHandler);
    }

    protected async showDialog(uri: URI): Promise<void> {
        const directory = await this.getDirectory(uri);
        const workspace = await this.workspace.workspace;
        const dialog = directory && await this.factory({ title: "Modules Types", destination: directory.uri });
        const typeId = dialog && await dialog.open();
        const configurations = workspace && typeId && directory && await this.server.createModules(typeId, directory.uri, { workspace: workspace.uri });

        if (configurations) return this.manager.addConfigurations(configurations);
    }
}
