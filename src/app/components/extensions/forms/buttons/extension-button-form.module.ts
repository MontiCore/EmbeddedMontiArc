/* (c) https://github.com/MontiCore/monticore */
import { NgModule } from "@angular/core";
import { ExtensionButtonFormComponent } from "./extension-button-form.component";
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

@NgModule({
    declarations: [ExtensionButtonFormComponent],
    imports: [
        MatIconModule, MatDividerModule, MatButtonModule, FormsModule, MatFormFieldModule,
        MatInputModule, BrowserModule, MatSelectModule, MatOptionModule, MatCheckboxModule,
        MatTooltipModule, MatMenuModule
    ],
    providers: [],
    exports: [ExtensionButtonFormComponent]
})
export class ExtensionButtonFormModule {}
