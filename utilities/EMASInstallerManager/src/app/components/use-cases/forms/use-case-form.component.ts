/* (c) https://github.com/MontiCore/monticore */
import { Component, Input } from "@angular/core";
import { UseCaseState } from "../../../services/use-cases/index";

@Component({
    selector: "app-use-case-form",
    templateUrl: "./use-case-form.component.html",
    styleUrls: ["./use-case-form.component.scss"]
})
export class UseCaseFormComponent {
    @Input() public readonly state: UseCaseState;

    public onFolderChange(event: Event): void {
        const target = event.target as any;
        const file = target.files[0];

        if (file) this.state.folder = file;
    }

    public onMainFileChange(event: Event): void {
        const target = event.target as any;
        const file = target.files[0];

        if (file) this.state.mainFile = file;
    }
}
