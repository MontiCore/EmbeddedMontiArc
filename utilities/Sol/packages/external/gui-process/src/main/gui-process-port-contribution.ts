/*
 * (c) https://github.com/MontiCore/monticore
 */
import { DockerPortContribution, DockerPortRegistry } from "@embeddedmontiarc/sol-external-docker/lib/main";
import { injectable } from "inversify";

@injectable()
export class GUIProcessPortContribution implements DockerPortContribution {
    public registerPorts(registry: DockerPortRegistry): void {
        registry.registerPort(10000);
    }
}
