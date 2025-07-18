/* (c) https://github.com/MontiCore/monticore */
import { NgModule } from "@angular/core";
import { UseCasesComponent } from "./use-cases.component";
import {
    MatButtonModule,
    MatCardModule,
    MatDividerModule,
    MatFormFieldModule,
    MatIconModule,
    MatInputModule,
    MatOptionModule, MatProgressBarModule,
    MatSelectModule, MatSnackBarModule, MatTooltipModule
} from "@angular/material";
import { CovalentLayoutModule } from "@covalent/core";
import { FormsModule } from "@angular/forms";
import { BrowserModule } from "@angular/platform-browser";
import { UseCaseFormModule } from "./forms";

@NgModule({
    declarations: [UseCasesComponent],
    imports: [
        MatCardModule, MatIconModule, MatDividerModule, CovalentLayoutModule, MatButtonModule, FormsModule, MatFormFieldModule,
        MatInputModule, BrowserModule, MatSelectModule, MatOptionModule, UseCaseFormModule, MatTooltipModule, MatProgressBarModule,
        MatSnackBarModule
    ],
    providers: [],
    exports: [UseCasesComponent]
})
export class UseCasesModule {}
