/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Configuration } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";
import { inject, injectable } from "inversify";
import { ModuleContext } from "../../common";
import { ModuleCreatorRegistry } from "./module-creator-registry";

export const ModuleCreatorDelegator = Symbol("ModuleCreatorDelegator");
export interface ModuleCreatorDelegator {
    createModules(typeId: string, destination: string, context: ModuleContext): Promise<Configuration[]>;
}

@injectable()
export class ModuleCreatorDelegatorImpl implements ModuleCreatorDelegator {
    @inject(ModuleCreatorRegistry) protected readonly registry: ModuleCreatorRegistry;

    public async createModules(typeId: string, destination: string, context: ModuleContext): Promise<Configuration[]> {
        const creator = this.registry.getModuleCreator(typeId);

        if (creator) return creator.createModules(destination, context);
        else throw new Error(`There is no module creator with id ${typeId}.`);
    }
}
