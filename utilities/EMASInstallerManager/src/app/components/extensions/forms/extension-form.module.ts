/* (c) https://github.com/MontiCore/monticore */
import { NgModule } from "@angular/core";
import { ExtensionFormComponent } from "./extension-form.component";
import {
    MatButtonModule, MatCheckboxModule,
    MatDividerModule,
    MatFormFieldModule,
    MatIconModule,
    MatInputModule, MatMenuModule,
    MatOptionModule,
    MatSelectModule, MatTooltipModule
} from "@angular/material";
import { FormsModule } from "@angular/forms";
import { BrowserModule } from "@angular/platform-browser";
import { ExtensionButtonFormModule } from "./buttons";

@NgModule({
    declarations: [ExtensionFormComponent],
    imports: [
        MatIconModule, MatDividerModule, MatButtonModule, FormsModule, MatFormFieldModule,
        MatInputModule, BrowserModule, MatSelectModule, MatOptionModule, MatCheckboxModule,
        MatTooltipModule, MatMenuModule, ExtensionButtonFormModule
    ],
    providers: [],
    exports: [ExtensionFormComponent]
})
export class ExtensionFormModule {}
