/* (c) https://github.com/MontiCore/monticore */
import { NgModule } from "@angular/core";
import { ControlScriptFormComponent } from "./control-script-form.component";
import {
    MatAutocompleteModule,
    MatButtonModule, MatCheckboxModule,
    MatDividerModule,
    MatFormFieldModule,
    MatIconModule,
    MatInputModule,
    MatOptionModule,
    MatSelectModule, MatTooltipModule
} from "@angular/material";
import { FormsModule } from "@angular/forms";
import { BrowserModule } from "@angular/platform-browser";

@NgModule({
    declarations: [ControlScriptFormComponent],
    imports: [
        MatIconModule, MatDividerModule, MatButtonModule, FormsModule, MatFormFieldModule,
        MatInputModule, BrowserModule, MatSelectModule, MatOptionModule, MatCheckboxModule,
        MatTooltipModule, MatAutocompleteModule
    ],
    providers: [],
    exports: [ControlScriptFormComponent]
})
export class ControlScriptFormModule {}
