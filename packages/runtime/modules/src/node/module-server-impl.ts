/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Configuration } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";
import { FileUri } from "@theia/core/lib/node";
import { inject, injectable } from "inversify";
import { ModuleServer } from "../common";
import { ModuleCreatorDelegator } from "./creator";

@injectable()
export class ModuleServerImpl implements ModuleServer {
    @inject(ModuleCreatorDelegator) protected readonly creator: ModuleCreatorDelegator;

    public async createModules(typeId: string, destination: string): Promise<Configuration[]> {
        return this.creator.createModules(typeId, FileUri.fsPath(destination));
    }
}
