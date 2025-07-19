/* (c) https://github.com/MontiCore/monticore */
import { NgModule } from "@angular/core";
import { ExtensionsComponent } from "./extensions.component";
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
import { ExtensionFormModule } from "./forms";

@NgModule({
    declarations: [ExtensionsComponent],
    imports: [
        MatCardModule, MatIconModule, MatDividerModule, CovalentLayoutModule, MatButtonModule, FormsModule, MatFormFieldModule,
        MatInputModule, BrowserModule, MatSelectModule, MatOptionModule, MatTooltipModule, ExtensionFormModule,
        MatSnackBarModule, MatProgressBarModule
    ],
    providers: [],
    exports: [ExtensionsComponent]
})
export class ExtensionsModule {}
