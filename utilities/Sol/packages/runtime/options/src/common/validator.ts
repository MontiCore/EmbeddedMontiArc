/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable, unmanaged } from "inversify";
import { OptionsContext } from "./option-protocol";

export interface Validator<V = any, E = any> { // tslint:disable-line:no-any
    readonly id: string;
    readonly type: string;

    validate(options: V, context: OptionsContext): Promise<E>;
}

@injectable()
export abstract class CommonValidator<V, E> implements Validator<V, E> {
    public readonly id: string;
    public readonly type: string;

    protected constructor(@unmanaged() id: string, @unmanaged() type: string) {
        this.id = id;
        this.type = type;
    }

    public abstract validate(options: V, context: OptionsContext): Promise<E>;
}
