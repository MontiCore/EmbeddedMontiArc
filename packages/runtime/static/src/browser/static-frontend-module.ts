/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContainerModule } from "inversify";
import { StaticService, StaticServiceImpl } from "./static-service";

export default new ContainerModule(bind => {
    bind(StaticServiceImpl).toSelf().inSingletonScope();
    bind(StaticService).toService(StaticServiceImpl);
});
