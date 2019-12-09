/*
 * (c) https://github.com/MontiCore/monticore
 */
export namespace StaticPaths {
    export const PATH: string = "/services/static";
}

export const StaticServer = Symbol("StaticServer");
export interface StaticServer {
    addStaticPath(uri: string): Promise<number>;
    removeStaticPath(id: number): Promise<void>;
}
