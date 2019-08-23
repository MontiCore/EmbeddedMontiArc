/* (c) https://github.com/MontiCore/monticore */
import { Injectable } from "@angular/core";
import { Artifact, ArtifactsService } from "./artifacts.service";
import { ArtifactsStateService, ArtifactState } from "./artifacts-state.service";

import * as fs from "fs-extra";

export class ArtifactStateTransformation {
    protected readonly state: ArtifactState;
    protected readonly service: ArtifactsService;

    public constructor(state: ArtifactState, service: ArtifactsService) {
        this.state = state;
        this.service = service;
    }

    public async apply(): Promise<Artifact> {
        const dependenciesFile = await this.service.getDependenciesFile();
        const dependencies = this.service.getArtifacts(true);
        const artifact = {
            comment: this.state.comment, platforms: this.state.platforms,
            from: this.state.from, to: this.state.to
        };

        dependencies.push(artifact);
        await fs.writeJSON(dependenciesFile, dependencies, { spaces: 4 });

        return artifact;
    }
}

@Injectable({ providedIn: "root" })
export class ArtifactsStateTransformerService {
    public constructor(
        protected readonly collection: ArtifactsService,
        protected readonly states: ArtifactsStateService
    ) {}

    public async apply(): Promise<void> {
        const states = this.states.getStates(true) as ArtifactState[];

        for (const state of states) {
            const transformation = new ArtifactStateTransformation(state, this.collection);
            const artifact = await transformation.apply();

            state.saved = true;

            this.collection.addArtifact(artifact);
        }
    }
}
