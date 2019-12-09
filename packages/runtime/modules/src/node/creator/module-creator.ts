/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Configuration } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";
import { injectable, inject, unmanaged } from "inversify";
import { ModuleCreatorDelegator } from "./module-creator-delegator";

export const ModuleCreator = Symbol("ModuleCreator");
export interface ModuleCreator {
    readonly id: string;

    createModules(destination: string): Promise<Configuration[]>;
}

@injectable()
export abstract class CommonModuleCreator implements ModuleCreator {
    @inject(ModuleCreatorDelegator) protected readonly creator: ModuleCreatorDelegator;

    public readonly id: string;

    protected constructor(@unmanaged() id: string) {
        this.id = id;
    }

    public abstract createModules(destination: string): Promise<Configuration[]>;
}
