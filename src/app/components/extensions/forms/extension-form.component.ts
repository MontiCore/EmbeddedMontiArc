/* (c) https://github.com/MontiCore/monticore */
import { Component, Input } from "@angular/core";
import { ButtonState, ExtensionState } from "@services/extensions";

@Component({
    selector: "app-extension-form",
    templateUrl: "./extension-form.component.html",
    styleUrls: ["./extension-form.component.scss"]
})
export class ExtensionFormComponent {
    @Input() public readonly state: ExtensionState;

    public removeButton(button: ButtonState) {
        this.state.removeButton(button);
    }
}
