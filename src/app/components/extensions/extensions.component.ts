/* (c) https://github.com/MontiCore/monticore */
import { Component } from "@angular/core";
import { ExtensionState, ExtensionsStateService } from "@services/extensions";
import { State, StateComponent } from "@services/common/state.service";
import { ExtensionsStateTransformer } from "@services/extensions/extensions-state-transformer";
import { MatSnackBar } from "@angular/material";
import { YarnService } from "@services/common/yarn.service";

@Component({
    selector: "app-extensions",
    templateUrl: "./extensions.component.html",
    styleUrls: ["./extensions.component.scss"]
})
export class ExtensionsComponent implements StateComponent {
    protected saving: boolean;

    public constructor(
        protected readonly service: ExtensionsStateService,
        protected readonly transformer: ExtensionsStateTransformer,
        protected readonly yarn: YarnService,
        protected readonly snackbar: MatSnackBar
    ) {
        this.saving = false;
    }

    public isSaving(): boolean {
        return this.saving;
    }

    public get states(): ExtensionState[] {
        return this.service.getStates() as ExtensionState[];
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
            .then(() => {
                this.snackbar.open("Linking Dependencies. Please be patient.", "OK");
                return this.yarn.install();
            })
            .then(() => this.snackbar.open("Extensions have successfully been added.", "OK"))
            .catch(error => this.snackbar.open(error.message, "OK"))
            .finally(() => this.saving = false);
    }
}
