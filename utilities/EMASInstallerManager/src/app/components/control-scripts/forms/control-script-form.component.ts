/* (c) https://github.com/MontiCore/monticore */
import { Component, Input } from "@angular/core";
import { ControlScriptState } from "@services/control-scripts";
import { UseCase, UseCasesService } from "@services/use-cases";
import { Extension, ExtensionsService } from "@services/extensions";

@Component({
    selector: "app-control-script-form",
    templateUrl: "./control-script-form.component.html",
    styleUrls: ["./control-script-form.component.scss"]
})
export class ControlScriptFormComponent {
    @Input() public readonly state: ControlScriptState;

    public constructor(
        protected readonly $useCases: UseCasesService,
        protected readonly $extensions: ExtensionsService
    ) {}

    public get useCases(): UseCase[] {
        return this.$useCases.getUseCasesWithPrefix(this.state.useCase);
    }

    public get extensions(): Extension[] {
        return this.$extensions.getExtensionsWithPrefix(this.state.extension);
    }

    public onFileChange(event: Event): void {
        const target = event.target as any;
        const file = target.files[0];

        if (file) this.state.path = file;
    }
}
