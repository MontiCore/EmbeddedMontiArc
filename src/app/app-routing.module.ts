/* (c) https://github.com/MontiCore/monticore */
import { NgModule } from "@angular/core";
import { Routes, RouterModule } from "@angular/router";
import { UseCasesComponent, WelcomeComponent, ArtifactsComponent, ControlScriptsComponent, ExtensionsComponent } from "@components/index";

const routes: Routes = [{
    path: '',
    component: WelcomeComponent
}, {
    path: "use-cases",
    component: UseCasesComponent
}, {
    path: "artifacts",
    component: ArtifactsComponent
}, {
    path: "control-scripts",
    component: ControlScriptsComponent
}, {
    path: "extensions",
    component: ExtensionsComponent
}];

@NgModule({
    imports: [RouterModule.forRoot(routes, { useHash: true })],
    exports: [RouterModule]
})
export class AppRoutingModule {}
