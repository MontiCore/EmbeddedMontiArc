/*
 * (c) https://github.com/MontiCore/monticore
 */
import { Configuration } from "@embeddedmontiarc/sol-runtime-configurations/lib/common";

export const MODULE_VALIDATOR_TYPE: string = "module";

export const MODULE_PATH: string = "/services/module";

export const ModuleServer = Symbol("ModuleServer");
export interface ModuleServer {
    createModules(typeId: string, destination: string): Promise<Configuration[]>;
}
