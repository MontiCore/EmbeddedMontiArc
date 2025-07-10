/* (c) https://github.com/MontiCore/monticore */
import { Component, Input } from "@angular/core";
import { ArtifactState, ArtifactsStateService } from "../../../services/artifacts/index";
import { MatCheckboxChange } from "@angular/material";

@Component({
    selector: "app-artifact-form",
    templateUrl: "./artifact-form.component.html",
    styleUrls: ["./artifact-form.component.scss"]
})
export class ArtifactFormComponent {
    @Input() public readonly state: ArtifactState;

    public constructor(protected readonly service: ArtifactsStateService) {}

    public get isForWindows(): boolean {
        return this.state.platforms.indexOf("win32") > -1;
    }

    public get isForLinux(): boolean {
        return this.state.platforms.indexOf("linux") > -1;
    }

    public onWindowsChange(event: MatCheckboxChange): void {
        const index = this.state.platforms.indexOf("win32");

        if (event.checked) this.state.platforms.push("win32");
        else this.state.platforms.splice(index, 1);
    }

    public onLinuxChange(event: MatCheckboxChange): void {
        const index = this.state.platforms.indexOf("linux");

        if (event.checked) this.state.platforms.push("linux");
        else this.state.platforms.splice(index, 1);
    }
}
