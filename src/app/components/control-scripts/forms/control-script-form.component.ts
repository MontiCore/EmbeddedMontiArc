import { Component, Input } from "@angular/core";
import { ControlScriptState } from "@services/control-scripts";

@Component({
    selector: "app-control-script-form",
    templateUrl: "./control-script-form.component.html",
    styleUrls: ["./control-script-form.component.scss"]
})
export class ControlScriptFormComponent {
    @Input() public readonly state: ControlScriptState;

    public onFileChange(event: Event): void {
        const target = event.target as any;
        const file = target.files[0];

        if (file) this.state.path = file;
    }
}
