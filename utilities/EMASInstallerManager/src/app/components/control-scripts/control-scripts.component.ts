/* (c) https://github.com/MontiCore/monticore */
import { Component } from "@angular/core";
import { ControlScriptsStateService } from "@services/control-scripts";
import { ControlScriptState } from "@services/control-scripts";
import { State, StateComponent } from "@services/common/state.service";
import { ControlScriptsStateTransformerService } from "@services/control-scripts/control-scripts-state-transformer.service";
import { MatSnackBar } from "@angular/material";

@Component({
    selector: "app-control-scripts",
    templateUrl: "./control-scripts.component.html",
    styleUrls: ["./control-scripts.component.scss"]
})
export class ControlScriptsComponent implements StateComponent {
    protected saving: boolean;

    public constructor(
        protected readonly service: ControlScriptsStateService,
        protected readonly transformer: ControlScriptsStateTransformerService,
        protected readonly snackbar: MatSnackBar
    ) {
        this.saving = false;
    }

    public get states(): ControlScriptState[] {
        return this.service.getStates() as ControlScriptState[];
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

    public isSaving(): boolean {
        return this.saving;
    }

    public apply(): void {
        this.saving = true;

        this.transformer.apply()
            .then(() => this.snackbar.open("Control Scripts have successfully been added.", "OK"))
            .catch(error => this.snackbar.open(error.message, "OK"))
            .finally(() => this.saving = false);
    }
}
