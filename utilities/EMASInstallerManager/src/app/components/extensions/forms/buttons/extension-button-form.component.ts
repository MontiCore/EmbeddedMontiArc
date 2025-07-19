/* (c) https://github.com/MontiCore/monticore */
import { Component, Input } from "@angular/core";
import { ButtonState } from "@services/extensions";

@Component({
    selector: "app-extension-button-form",
    templateUrl: "./extension-button-form.component.html",
    styleUrls: ["./extension-button-form.component.scss"]
})
export class ExtensionButtonFormComponent {
    @Input() public readonly state: ButtonState;

    public onImageChange(event: Event): void {
        const target = event.target as any;
        const file = target.files[0];

        if (file) this.state.image = file;
    }
}
