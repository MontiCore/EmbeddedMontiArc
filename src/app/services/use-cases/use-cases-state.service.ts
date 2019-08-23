/* (c) https://github.com/MontiCore/monticore */
import { Platform } from "@services/common/platform";
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

        messages.set(validators.getExistenceOfValueValidator(), "The name of the model in UpperCamelCase (e.g AutoPilot).");
        messages.set(validators.getUpperCamelCaseValidator(), "The name should be in UpperCamelCase.");
        messages.set(validators.getAlphanumericalValidator(), "The name should be alphanumerical.");
        messages.set(validators.getUniqueUseCaseValidator(), "There is already a use case with this name.");

        return messages;
    }
}

class ValidatedPlatform extends ValidatedValue<Platform> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("platform", "all", errors, validators);
    }
}

class ValidatedMainFile extends ValidatedValue<File | undefined> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("mainFile", undefined, errors, validators);
    }
}

class ValidatedFolder extends ValidatedValue<File | undefined> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("folder", undefined, errors, validators);
    }

    protected fetchValidators(validators: ValidatorsService): ValidatorErrorMap {
        const messages = new Map();

        messages.set(validators.getExistenceOfFileValidator(), '');

        return messages;
    }
}

export class UseCaseState extends AbstractState {
    protected _name: ValidatedName;
    protected _platform: ValidatedPlatform;
    protected _mainFile: ValidatedMainFile;
    protected _folder: ValidatedFolder;

    public constructor(validators: ValidatorsService) {
        super();
        this._name = new ValidatedName(this.errors, validators);
        this._platform = new ValidatedPlatform(this.errors, validators);
        this._mainFile = new ValidatedMainFile(this.errors, validators);
        this._folder = new ValidatedFolder(this.errors, validators);
    }

    public get name(): string {
        return this._name.get();
    }

    public set name(name: string) {
        this._name.set(name);
    }

    public get platform(): Platform {
        return this._platform.get();
    }

    public set platform(platform: Platform) {
        this._platform.set(platform);
    }

    public get mainFile(): File | undefined {
        return this._mainFile.get();
    }

    public set mainFile(mainFile: File | undefined) {
        this._mainFile.set(mainFile);
    }

    public get folder(): File | undefined {
        return this._folder.get();
    }

    public set folder(folder: File | undefined) {
        this._folder.set(folder);
    }
}

@Injectable({ providedIn: "root" })
export class UseCasesStateService extends AbstractStateService {
    public constructor(protected readonly validators: ValidatorsService) {
        super();
    }

    public getNewState(): State {
        return new UseCaseState(this.validators);
    }
}
