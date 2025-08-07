/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Configuration } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";
import { FileUri } from "@theia/core/lib/node";
import { injectable, inject, unmanaged } from "inversify";
import { ModuleContext } from "../../common";
import { ModuleCreatorDelegator } from "./module-creator-delegator";

import * as path from "path";

export const ModuleCreator = Symbol("ModuleCreator");
export interface ModuleCreator {
    readonly id: string;

    createModules(destination: string, context: ModuleContext): Promise<Configuration[]>;
}

@injectable()
export abstract class CommonModuleCreator implements ModuleCreator {
    @inject(ModuleCreatorDelegator) protected readonly creator: ModuleCreatorDelegator;

    public readonly id: string;

    protected constructor(@unmanaged() id: string) {
        this.id = id;
    }

    public abstract createModules(destination: string, context: ModuleContext): Promise<Configuration[]>;

    protected relative(context: ModuleContext, destination: string, relativePath: string): string {
        if (context.workspace) return path.relative(FileUri.fsPath(context.workspace), path.resolve(destination, relativePath));
        return relativePath;
    }
}
