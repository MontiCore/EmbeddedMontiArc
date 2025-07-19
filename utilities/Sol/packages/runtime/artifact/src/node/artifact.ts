/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable, unmanaged } from "inversify";

export interface Artifact {
    readonly path: string;
}

@injectable()
export class CommonArtifact implements Artifact {
    public readonly path: string;

    protected constructor(@unmanaged() path: string) {
        this.path = path;
    }
}
