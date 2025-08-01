/*
 * (c) https://github.com/MontiCore/monticore
 */
import { ContainerModule } from "inversify";
import { bindCommon } from "../common/core-common-module";

export default new ContainerModule(bind => {
    bindCommon(bind);
});
