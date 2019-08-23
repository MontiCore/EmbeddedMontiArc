/* (c) https://github.com/MontiCore/monticore */
import { URL } from "url";
import { ValidationErrors } from "@angular/forms";
import { Injectable } from "@angular/core";
import { ExtensionsService } from "@services/extensions/extensions.service";
import { UseCasesService } from "@services/use-cases/use-cases.service";

import * as path from "path";

export type ValidatorFunction = (value: any) => boolean;
export type ValidatorErrorMap = Map<ValidatorFunction, string>;

export abstract class ValidatedValue<T> {
    protected readonly id: string;
    protected readonly validators: ValidatorErrorMap;
    protected readonly errors: ValidationErrors;

    protected value: T;

    protected constructor(id: string, value: T, errors: ValidationErrors, validators: ValidatorsService) {
        this.id = id;
        this.validators = this.fetchValidators(validators);
        this.errors = errors;

        this.set(value);
    }

    protected fetchValidators(validators: ValidatorsService): ValidatorErrorMap {
        return new Map();
    }

    public set(value: T): void {
        this.value = value;

        this.validate();
    }

    public get(): T {
        return this.value;
    }

    protected validate(): void {
        const validators = this.validators.keys();

        delete this.errors[this.id];

        for (const validate of validators) {
            if (!validate(this.value)) {
                this.errors[this.id] = this.validators.get(validate);
                break;
            }
        }
    }
}

@Injectable({ providedIn: "root" })
export class ValidatorsService {
    public constructor(
        protected readonly useCases: UseCasesService,
        protected readonly extensions: ExtensionsService
    ) {}

    protected isUpperCase(value: string): boolean {
        let result = true;

        for (const character of value) {
            result = result && character === character.toUpperCase();
        }

        return result;
    }

    protected isLowerCase(value: string): boolean {
        let result = true;

        for (const character of value) {
            result = result && character === character.toLowerCase();
        }

        return result;
    }

    public getUniqueExtensionValidator(): ValidatorFunction {
        return (value: string) => {
            const name = "@emastudio/" + value.toLowerCase();
            const extensions = this.extensions.getExtensions();

            for (const extension of extensions) {
                if (extension.name === name) return false;
            }

            return true;
        };
    }

    public getUniqueUseCaseValidator(): ValidatorFunction {
        return (value: string) => {
            const useCases = this.useCases.getUseCases();

            for (const useCase of useCases) {
                if (useCase.name === value) return false;
            }

            return true;
        };
    }

    public getExtensionValidator(): ValidatorFunction {
        return (value: string) => {
            const name = "@emastudio/" + value.toLowerCase();
            const extensions = this.extensions.getExtensions();

            for (const extension of extensions) {
                if (extension.name === name) return true;
            }

            return false;
        };
    }

    public getUseCaseValidator(): ValidatorFunction {
        return (value: string) => {
            const useCases = this.useCases.getUseCases();

            for (const useCase of useCases) {
                if (useCase.name === value) return true;
            }

            return false;
        };
    }

    public getNoWhitespacesValidator(): ValidatorFunction {
        return (value: string) => {
            const parts = value.split(' ');

            return parts.length === 1;
        };
    }

    public getAlphanumericalValidator(): ValidatorFunction {
        return (value: string) => {
            const pattern = /^[a-zA-Z0-9]+$/i;
            const matches = value.match(pattern);

            return matches !== null;
        };
    }

    public getDotNotationValidator(): ValidatorFunction {
        const validateNoWhitespaces = this.getNoWhitespacesValidator();

        return (value: string) => {
            const noWhitespaces = validateNoWhitespaces(value);
            const isLowerCase = this.isLowerCase(value);

            return noWhitespaces && isLowerCase;
        };
    }

    public getExistenceOfValueValidator(): ValidatorFunction {
        return (value: string) => {
            return value.length > 0;
        };
    }

    public getExistenceOfFileValidator(): ValidatorFunction {
        return (value: File | undefined) => {
            return value !== undefined;
        };
    }

    public getURLValidator(): ValidatorFunction {
        return (value: string) => {
            // tslint:disable-next-line:no-unused-expression
            try { new URL(value); } catch (error) { return false; }
            return true;
        };
    }

    public getPathValidator(): ValidatorFunction {
        return (value: string) => {
            try { path.posix.parse(value); } catch (error) { return false; }
            return true;
        };
    }

    public getUpperCamelCaseValidator(): ValidatorFunction {
        const validateNoWhitespaces = this.getNoWhitespacesValidator();

        return (value: string) => {
            return this.isUpperCase(value.charAt(0)) && validateNoWhitespaces(value);
        };
    }
}
