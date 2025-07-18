/* (c) https://github.com/MontiCore/monticore */
import { NgModule } from "@angular/core";
import { ControlScriptsComponent } from "./control-scripts.component";
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
import { ControlScriptFormModule } from "./forms";

@NgModule({
    declarations: [ControlScriptsComponent],
    imports: [
        MatCardModule, MatIconModule, MatDividerModule, CovalentLayoutModule, MatButtonModule, FormsModule, MatFormFieldModule,
        MatInputModule, BrowserModule, MatSelectModule, MatOptionModule, ControlScriptFormModule, MatTooltipModule,
        MatProgressBarModule, MatSnackBarModule
    ],
    providers: [],
    exports: [ControlScriptsComponent]
})
export class ControlScriptsModule {}
