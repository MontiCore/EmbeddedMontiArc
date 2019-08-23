/* (c) https://github.com/MontiCore/monticore */
import { Component } from "@angular/core";
import { UseCaseState, UseCasesStateService } from "@services/use-cases";
import { State, StateComponent } from "@services/common/state.service";
import { UseCasesStateTransformerService } from "@services/use-cases/use-cases-state-transformer.service";
import { MatSnackBar } from "@angular/material";

@Component({
    selector: "app-use-cases",
    templateUrl: "./use-cases.component.html",
    styleUrls: ["./use-cases.component.scss"]
})
export class UseCasesComponent implements StateComponent {
    protected saving: boolean;

    public constructor(
        protected readonly service: UseCasesStateService,
        protected readonly transformer: UseCasesStateTransformerService,
        protected readonly snackbar: MatSnackBar
    ) {
        this.saving = false;
    }

    public get states(): UseCaseState[] {
        return this.service.getStates() as UseCaseState[];
    }

    public isSaving(): boolean {
        return this.saving;
    }

    public hasErrors(): boolean {
        return this.service.hasErrors();
    }

    public hasStates(): boolean {
        return this.service.hasStates();
    }

    public addState(): void {
        this.service.addState();
    }

    public removeState(state: State): void {
        this.service.removeState(state);
    }

    public apply(): void {
        this.saving = true;

        this.transformer.apply()
            .then(() => this.snackbar.open("Use Cases have successfully been added.", "OK"))
            .catch(error => this.snackbar.open(error.message, "OK"))
            .finally(() => this.saving = false);
    }
}
