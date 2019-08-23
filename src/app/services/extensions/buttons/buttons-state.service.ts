/* (c) https://github.com/MontiCore/monticore */
import { ValidatedValue, ValidatorErrorMap, ValidatorsService } from "@services/common/validators.service";
import { AbstractState } from "@services/common/state.service";
import { ValidationErrors } from "@angular/forms";

class ValidatedLabel extends ValidatedValue<string> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("label", '', errors, validators);
    }

    protected fetchValidators(validators: ValidatorsService): ValidatorErrorMap {
        const messages = new Map();

        messages.set(validators.getExistenceOfValueValidator(), "The label standing besides the icon of the button.");

        return messages;
    }
}

class ValidatedScript extends ValidatedValue<string> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("script", '', errors, validators);
    }

    protected fetchValidators(validators: ValidatorsService): ValidatorErrorMap {
        const messages = new Map();

        messages.set(validators.getExistenceOfValueValidator(), `The name of the script which should be executed when
            the button is clicked.`);
        messages.set(validators.getDotNotationValidator(), `The name should be in lower case dot separated notation
            (e.g execute.distributed.bat).`);

        return messages;
    }
}

class ValidatedImage extends ValidatedValue<File | undefined> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("image", undefined, errors, validators);
    }

    protected fetchValidators(validators: ValidatorsService): ValidatorErrorMap {
        const messages = new Map();

        messages.set(validators.getExistenceOfFileValidator(), '');

        return messages;
    }
}

export class ButtonState extends AbstractState {
    protected _label: ValidatedLabel;
    protected _script: ValidatedScript;
    protected _image: ValidatedImage;

    public constructor(validators: ValidatorsService) {
        super();
        this._label = new ValidatedLabel(this.errors, validators);
        this._script = new ValidatedScript(this.errors, validators);
        this._image = new ValidatedImage(this.errors, validators);
    }

    public get label(): string {
        return this._label.get();
    }

    public set label(id: string) {
        this._label.set(id);
    }

    public get script(): string {
        return this._script.get();
    }

    public set script(script: string) {
        this._script.set(script);
    }

    public get image(): File | undefined {
        return this._image.get();
    }

    public set image(image: File | undefined) {
        this._image.set(image);
    }
}
