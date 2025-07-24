/*
 * (c) https://github.com/MontiCore/monticore
 */
import { RouteContribution } from "@embeddedmontiarc/sol-external-core/lib/renderer/router-fragment";
import { ContainerModule } from "inversify";
import { Preparations } from "./preparations";

export default new ContainerModule(bind => {
    bind(RouteContribution).toConstantValue({
        path: "/",
        component: Preparations
    });
});
