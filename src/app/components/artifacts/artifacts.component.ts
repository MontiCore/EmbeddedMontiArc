/* (c) https://github.com/MontiCore/monticore */
import { Component } from "@angular/core";
import { ArtifactState, ArtifactsStateService } from "@services/artifacts/artifacts-state.service";
import { State, StateComponent } from "@services/common/state.service";
import { ArtifactsStateTransformerService } from "@services/artifacts/artifacts-state-transformer.service";
import { MatSnackBar } from "@angular/material";
import { YarnService } from "@services/common/yarn.service";

@Component({
    selector: "app-artifacts",
    templateUrl: "./artifacts.component.html",
    styleUrls: ["./artifacts.component.scss"]
})
export class ArtifactsComponent implements StateComponent {
    protected saving: boolean;

    public constructor(
        protected readonly service: ArtifactsStateService,
        protected readonly transformer: ArtifactsStateTransformerService,
        protected readonly yarn: YarnService,
        protected readonly snackbar: MatSnackBar
    ) {
        this.saving = false;
    }

    public get states(): ArtifactState[] {
        return this.service.getStates() as ArtifactState[];
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
            .then(() => {
                this.snackbar.open("Downloading Dependencies. Please be patient.", "OK");
                return this.yarn.install();
            })
            .then(() => this.snackbar.open("Artifacts have successfully been added.", "OK"))
            .catch(error => this.snackbar.open(error.message, "OK"))
            .finally(() => this.saving = false);
    }
}
