/* (c) https://github.com/MontiCore/monticore */
import { BrowserModule } from "@angular/platform-browser";
import { NgModule } from "@angular/core";

import { AppRoutingModule } from "./app-routing.module";
import { AppComponent } from "./app.component";

import { BrowserAnimationsModule } from "@angular/platform-browser/animations";
import { CovalentCommonModule, CovalentLayoutModule, CovalentTabSelectModule } from "@covalent/core";

import {
    MatButtonModule, MatCardModule, MatIconModule, MatListModule, MatSidenavModule, MatSnackBarModule,
    MatToolbarModule, MatTooltipModule, MatTabsModule, MatProgressSpinnerModule
} from "@angular/material";
import { WelcomeModule, UseCasesModule, ArtifactsModule, ControlScriptsModule, ExtensionsModule } from "@components/index";

const materialModules = [
    MatToolbarModule,
    MatIconModule,
    MatButtonModule,
    MatListModule,
    MatCardModule,
    MatSnackBarModule,
    MatSidenavModule,
    MatTooltipModule,
    MatTabsModule,
    MatProgressSpinnerModule
];

const covalentModules = [
    CovalentCommonModule,
    CovalentLayoutModule,
    CovalentTabSelectModule
];

const componentModules = [
    WelcomeModule,
    UseCasesModule,
    ArtifactsModule,
    ControlScriptsModule,
    ExtensionsModule
];

const imports = [
    ...materialModules,
    ...covalentModules,
    ...componentModules,
    BrowserModule,
    AppRoutingModule,
    BrowserAnimationsModule
];

const declarations = [
    AppComponent
];

const exports = [
    ...imports,
    ...declarations
];

@NgModule({
    declarations: declarations,
    imports: imports,
    exports: exports,
    providers: [],
    bootstrap: [AppComponent]
})
export class AppModule {}
