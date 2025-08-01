/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";

export interface Product {
    readonly path: string;
}

@injectable()
export class CommonProduct implements Product {
    public readonly path: string;

    protected constructor(path: string) {
        this.path = path;
    }
}
