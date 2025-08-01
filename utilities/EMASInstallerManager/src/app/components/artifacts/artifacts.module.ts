/* (c) https://github.com/MontiCore/monticore */
import { NgModule } from "@angular/core";
import { ArtifactsComponent } from "./artifacts.component";
import {
    MatButtonModule,
    MatCardModule,
    MatDividerModule,
    MatFormFieldModule,
    MatIconModule,
    MatInputModule, MatOptionModule, MatProgressBarModule,
    MatSelectModule, MatSnackBarModule, MatTooltipModule
} from "@angular/material";
import { CovalentLayoutModule } from "@covalent/core";
import { FormsModule } from "@angular/forms";
import { BrowserModule } from "@angular/platform-browser";
import { ArtifactFormModule } from "@components/artifacts/forms/index";

@NgModule({
    declarations: [ArtifactsComponent],
    imports: [
        MatCardModule, MatIconModule, MatDividerModule, CovalentLayoutModule, MatButtonModule, FormsModule, MatFormFieldModule,
        MatInputModule, BrowserModule, MatSelectModule, MatOptionModule, ArtifactFormModule, MatTooltipModule, MatSnackBarModule,
        MatProgressBarModule
    ],
    providers: [],
    exports: [ArtifactsComponent]
})
export class ArtifactsModule {}
