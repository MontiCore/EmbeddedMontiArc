/* (c) https://github.com/MontiCore/monticore */
import { NgModule } from "@angular/core";
import { ArtifactFormComponent } from "./artifact-form.component";
import {
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
    declarations: [ArtifactFormComponent],
    imports: [
        MatIconModule, MatDividerModule, MatButtonModule, FormsModule, MatFormFieldModule,
        MatInputModule, BrowserModule, MatSelectModule, MatOptionModule, MatCheckboxModule,
        MatTooltipModule
    ],
    providers: [],
    exports: [ArtifactFormComponent]
})
export class ArtifactFormModule {}
