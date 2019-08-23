/* (c) https://github.com/MontiCore/monticore */
import { TemplatesContribution } from "@embeddedmontiarc/sol-runtime-templates/lib/node/templates-contribution";
import { ContainerModule } from "inversify";
import { TestTemplatesContribution } from "./test-templates-contribution";

export default new ContainerModule(bind => {
    bind(TemplatesContribution).to(TestTemplatesContribution).inSingletonScope();
});
