/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";
import { AbstractState, AbstractStateService, State } from "@services/common/state.service";
import { ValidatedValue, ValidatorErrorMap, ValidatorsService } from "@services/common/validators.service";
import { ButtonState } from "@services/extensions/buttons";
import { ValidationErrors } from "@angular/forms";

class ValidatedName extends ValidatedValue<string> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("name", '', errors, validators);
    }

    protected fetchValidators(validators: ValidatorsService): ValidatorErrorMap {
        const messages = new Map();

        messages.set(validators.getExistenceOfValueValidator(), "The name of the extension in UpperCamelCase (e.g AutoPilot).");
        messages.set(validators.getUpperCamelCaseValidator(), "The name should be in UpperCamelCase.");
        messages.set(validators.getAlphanumericalValidator(), "The name should be alphanumerical.");
        messages.set(validators.getUniqueExtensionValidator(), "There is already an extension with this name.");

        return messages;
    }
}

export class ExtensionState extends AbstractState {
    protected readonly validators: ValidatorsService;

    protected _name: ValidatedName;
    protected _buttons: ButtonState[];

    public constructor(validators: ValidatorsService) {
        super();
        this.validators = validators;

        this._name = new ValidatedName(this.errors, this.validators);
        this._buttons = [];
    }

    public hasError(key: string): boolean {
        let hasError = super.hasError(key);

        for (const button of this._buttons) {
            hasError = hasError || button.hasError(key);
        }

        return hasError;
    }

    public hasAnyError(): boolean {
        let hasAnyError = super.hasAnyError();

        for (const button of this._buttons) {
            hasAnyError = hasAnyError || button.hasAnyError();
        }

        return hasAnyError;
    }

    public get name(): string {
        return this._name.get();
    }

    public set name(name: string) {
        this._name.set(name);
    }

    public getButtons(unsaved: boolean = false): ButtonState[] {
        return unsaved ? this._buttons.filter(state => !state.saved) : this._buttons;
    }

    public addButton(): void {
        this._buttons.push(new ButtonState(this.validators));
    }

    public removeButton(state: ButtonState): void {
        const index = this._buttons.indexOf(state);

        this._buttons.splice(index, 1);
    }
}

@Injectable({ providedIn: "root" })
export class ExtensionsStateService extends AbstractStateService {
    public constructor(protected readonly validators: ValidatorsService) {
        super();
    }

    public getNewState(): State {
        return new ExtensionState(this.validators);
    }
}
