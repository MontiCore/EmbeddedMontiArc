/*
 * (c) https://github.com/MontiCore/monticore
 */
declare module "hasbin" {
    function hasbin(executable: string, handler: (result: boolean) => void): void;

    namespace hasbin {
        export function all(executables: string[], handler: (result: boolean) => void): void;
        export function some(executables: string[], handler: (result: boolean) => void): void;
        export function first(executables: string[], handler: (result: string | boolean) => void): void;
    }

    export = hasbin;
}
