/* (c) https://github.com/MontiCore/monticore */
import { Platform } from "@services/common/platform";
import { Injectable } from "@angular/core";
import { AbstractState, AbstractStateService, State } from "@services/common/state.service";
import { ValidatedValue, ValidatorErrorMap, ValidatorsService } from "@services/common/validators.service";
import { ValidationErrors } from "@angular/forms";

class ValidatedComment extends ValidatedValue<string> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("comment", '', errors, validators);
    }
}

class ValidatedPlatforms extends ValidatedValue<Platform[]> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("platforms", [], errors, validators);
    }
}

class ValidatedFrom extends ValidatedValue<string> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("from", '', errors, validators);
    }

    protected fetchValidators(validators: ValidatorsService): ValidatorErrorMap {
        const messages = new Map();

        messages.set(validators.getExistenceOfValueValidator(), "The URL from which the ZIP archive should be downloaded.");
        messages.set(validators.getURLValidator(), "This URL is not valid.");

        return messages;
    }
}

class ValidatedTo extends ValidatedValue<string> {
    public constructor(errors: ValidationErrors, validators: ValidatorsService) {
        super("to", "resources", errors, validators);
    }

    protected fetchValidators(validators: ValidatorsService): ValidatorErrorMap {
        const messages = new Map();

        messages.set(validators.getPathValidator(), "The specified path is not valid.");

        return messages;
    }
}

export class ArtifactState extends AbstractState {
    protected _comment: ValidatedComment;
    protected _platforms: ValidatedPlatforms;
    protected _from: ValidatedFrom;
    protected _to: ValidatedTo;

    public constructor(validators: ValidatorsService) {
        super();
        this._comment = new ValidatedComment(this.errors, validators);
        this._platforms = new ValidatedPlatforms(this.errors, validators);
        this._from = new ValidatedFrom(this.errors, validators);
        this._to = new ValidatedTo(this.errors, validators);
    }

    public get comment(): string {
        return this._comment.get();
    }

    public set comment(comment: string) {
        this._comment.set(comment);
    }

    public get platforms(): Platform[] {
        return this._platforms.get();
    }

    public set platforms(platforms: Platform[]) {
        this._platforms.set(platforms);
    }

    public get from(): string {
        return this._from.get();
    }

    public set from(from: string) {
        this._from.set(from);
    }

    public get to(): string {
        return this._to.get();
    }

    public set to(to: string) {
        this._to.set(to);
    }
}

@Injectable({ providedIn: "root" })
export class ArtifactsStateService extends AbstractStateService {
    public constructor(protected readonly validators: ValidatorsService) {
        super();
    }

    public getNewState(): State {
        return new ArtifactState(this.validators);
    }
}
