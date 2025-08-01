/*
 * (c) https://github.com/MontiCore/monticore
 */
import { inject, injectable } from "inversify";
import { ValidatorServer, ValidatorRegistry, OptionsContext } from "../common";

@injectable()
export class ValidatorServerImpl implements ValidatorServer {
    @inject(ValidatorRegistry) protected readonly registry: ValidatorRegistry;

    public async validate<V, E>(id: string, type: string, options: V, context: OptionsContext): Promise<E> {
        const validator = this.registry.getValidator(id, type);

        if (validator) return validator.validate(options, context);
        else return {} as E;
    }
}
