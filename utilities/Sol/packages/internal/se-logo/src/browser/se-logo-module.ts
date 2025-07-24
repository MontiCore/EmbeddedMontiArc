/* (c) https://github.com/MontiCore/monticore */

import { FrontendApplicationContribution } from "@theia/core/lib/browser";
import { ContainerModule } from "inversify";
import { SELogoContribution } from "./se-logo-contribution";

import "../../src/browser/style/index.css";

export default new ContainerModule(bind => {
    bind(SELogoContribution).toSelf().inSingletonScope();
    bind(FrontendApplicationContribution).toService(SELogoContribution);
});
