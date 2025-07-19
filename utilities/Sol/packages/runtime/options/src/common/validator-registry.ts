/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContextContainer } from "@embeddedmontiarc/sol-runtime-core/lib/common";
import { ContributionProvider } from "@theia/core/lib/common";
import { memo } from "helpful-decorators";
import { inject, injectable, interfaces, named } from "inversify";
import { Validator } from "./validator";

import ServiceIdentifier = interfaces.ServiceIdentifier;
import Container = interfaces.Container;

export const ValidatorContribution = Symbol("ValidatorContribution");
export interface ValidatorContribution {
    registerValidators(registry: ValidatorRegistry): void;
}

export const ValidatorRegistry = Symbol("ValidatorRegistry");
export interface ValidatorRegistry {
    registerValidator(id: ServiceIdentifier<Validator>): void;
    unregisterValidator(id: string, type: string): void;
    getValidators(): Validator[];
    getValidator(id: string, type: string): Validator | undefined;
}

@injectable()
export class CommonValidatorRegistry implements ValidatorRegistry {
    @inject(ContributionProvider) @named(ValidatorContribution)
    protected readonly provider: ContributionProvider<ValidatorContribution>;

    @inject(ContextContainer) protected readonly container: Container;
    protected readonly validators: Map<string, Validator>;

    public constructor() {
        this.validators = new Map();
    }

    public registerValidator(id: ServiceIdentifier<Validator>): void {
        const validator = this.container.get(id);
        const key = this.computeKey(validator.id, validator.type);

        if (this.validators.has(key)) throw new Error(`There is already a validator registered under the key of ${key}.`);
        else this.validators.set(key, validator);
    }

    public unregisterValidator(id: string, type: string): void {
        const key = this.computeKey(id, type);

        if (this.validators.has(key)) this.validators.delete(key);
        else console.warn(`There is no validator registered under the key of ${key}.`);
    }

    @memo()
    public getValidator(id: string, type: string): Validator | undefined {
        return this.validators.get(this.computeKey(id, type));
    }

    @memo()
    public getValidators(): Validator[] {
        return [...this.validators.values()];
    }

    @memo()
    protected computeKey(id: string, type: string): string {
        return `${type}:${id}`;
    }
}
