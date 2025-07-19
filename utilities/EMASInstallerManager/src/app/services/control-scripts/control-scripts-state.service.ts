/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";
import { AbstractState, AbstractStateService, State } from "@services/common/state.service";
import { ValidatedValue, ValidatorErrorMap, ValidatorsService } from "@services/common/validators.service";
import { ValidationErrors } from "@angular/forms";

class ValidatedName extends ValidatedValue<string> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("name", '', errors, validators);
    }

    protected fetchValidators(validators: ValidatorsService): ValidatorErrorMap {
        const messages = new Map();

        messages.set(validators.getExistenceOfValueValidator(), `The name of the script in lower case dot separated notation
            (e.g execute.distributed.bat).`);
        messages.set(validators.getDotNotationValidator(), `The name of the script in lower case dot separated notation
            (e.g execute.distributed.bat).`);

        return messages;
    }
}

class ValidatedUseCase extends ValidatedValue<string> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("useCase", '', errors, validators);
    }

    protected fetchValidators(validators: ValidatorsService): ValidatorErrorMap {
        const messages = new Map();

        messages.set(validators.getExistenceOfValueValidator(), "Use Case to which the script should be added.");
        messages.set(validators.getUseCaseValidator(), "There is no such use case.");

        return messages;
    }
}

class ValidatedExtension extends ValidatedValue<string> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("extension", '', errors, validators);
    }

    protected fetchValidators(validators: ValidatorsService): ValidatorErrorMap {
        const messages = new Map();

        messages.set(validators.getExistenceOfValueValidator(), "Extension to which the script should be added.");
        messages.set(validators.getExtensionValidator(), "There is no such extension.");

        return messages;
    }
}

class ValidatedPath extends ValidatedValue<File | undefined> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("path", undefined, errors, validators);
    }

    protected fetchValidators(validators: ValidatorsService): ValidatorErrorMap {
        const messages = new Map();

        messages.set(validators.getExistenceOfFileValidator(), '');

        return messages;
    }
}

export class ControlScriptState extends AbstractState {
    protected _name: ValidatedName;
    protected _useCase: ValidatedUseCase;
    protected _extension: ValidatedExtension;
    protected _path: ValidatedPath;

    public constructor(validators: ValidatorsService) {
        super();
        this._name = new ValidatedName(this.errors, validators);
        this._useCase = new ValidatedUseCase(this.errors, validators);
        this._extension = new ValidatedExtension(this.errors, validators);
        this._path = new ValidatedPath(this.errors, validators);
    }

    public get name(): string {
        return this._name.get();
    }

    public set name(name: string) {
        this._name.set(name);
    }

    public get useCase(): string {
        return this._useCase.get();
    }

    public set useCase(useCase: string) {
        this._useCase.set(useCase);
    }

    public get extension(): string {
        return this._extension.get();
    }

    public set extension(extension: string) {
        this._extension.set(extension);
    }

    public get path(): File | undefined {
        return this._path.get();
    }

    public set path(path: File | undefined) {
        this._path.set(path);
    }
}

@Injectable({ providedIn: "root" })
export class ControlScriptsStateService extends AbstractStateService {
    public constructor(protected readonly validators: ValidatorsService) {
        super();
    }

    public getNewState(): State {
        return new ControlScriptState(this.validators);
    }
}
