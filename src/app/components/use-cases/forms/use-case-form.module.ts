/* (c) https://github.com/MontiCore/monticore */
import { NgModule } from "@angular/core";
import { UseCaseFormComponent } from "./use-case-form.component";
import {
    MatButtonModule,
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
    declarations: [UseCaseFormComponent],
    imports: [
        MatIconModule, MatDividerModule, MatButtonModule, FormsModule, MatFormFieldModule,
        MatInputModule, BrowserModule, MatSelectModule, MatOptionModule, MatTooltipModule
    ],
    providers: [],
    exports: [UseCaseFormComponent]
})
export class UseCaseFormModule {}
