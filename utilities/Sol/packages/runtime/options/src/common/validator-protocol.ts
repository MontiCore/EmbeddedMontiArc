/*
 * (c) https://github.com/MontiCore/monticore
 */
import { OptionsContext } from "./option-protocol";

export namespace ValidatorPaths {
    export const PATH: string = "/services/validators";
}

export const ValidatorServer = Symbol("ValidatorServer");
export interface ValidatorServer {
    validate<V, E>(id: string, type: string, options: V, context: OptionsContext): Promise<E>;
}
