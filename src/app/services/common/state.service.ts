import { AbstractControl, ValidationErrors } from "@angular/forms";

export interface State {
    hasError(key: string): boolean;
    hasErrorWithControl(key: string, control: AbstractControl): boolean;
    hasAnyError(): boolean;
    getError(key: string): string;
}

export interface StateService {
    hasErrors(): boolean;
    hasStates(): boolean;
    addState(): void;
    removeState(state: State): void;
}

export interface StateComponent {
    states: State[];
    hasErrors(): boolean;
    hasStates(): boolean;
    addState(): void;
    removeState(state: State): void;
}

export abstract class AbstractState implements State {
    protected readonly errors: ValidationErrors;

    protected constructor() {
        this.errors = {};
    }

    public hasError(key: string): boolean {
        return this.errors.hasOwnProperty(key);
    }

    public hasErrorWithControl(key: string, control: AbstractControl): boolean {
        const hasError = this.hasError(key);

        if (hasError) control.setErrors({ "hasErrors": true });
        else control.setErrors(null);

        return hasError;
    }

    public hasAnyError(): boolean {
        const keys = Object.keys(this.errors);

        return keys.length > 0;
    }

    public setError(key: string, message: string): void {
        this.errors[key] = message;
    }

    public getError(key: string): string {
        return this.errors[key];
    }

    public unsetError(key: string): void {
        delete this.errors[key];
    }
}

export abstract class AbstractStateService implements StateService {
    protected readonly states: State[];

    protected constructor() {
        this.states = [];
    }

    public getStates(copy: boolean = false): State[] {
        return copy ? this.states.slice(0) : this.states;
    }

    public hasErrors(): boolean {
        for (const state of this.states) {
            if (state.hasAnyError()) return true;
        }

        return false;
    }

    public hasStates(): boolean {
        return this.states.length > 0;
    }

    protected abstract getNewState(): State;

    public addState(): void {
        this.states.push(this.getNewState());
    }

    public removeState(state: State): void {
        const index = this.states.indexOf(state);

        this.states.splice(index, 1);
    }
}
