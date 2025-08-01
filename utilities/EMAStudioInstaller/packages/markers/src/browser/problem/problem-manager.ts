/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { injectable } from "inversify";
import { MarkerCollection, ProblemManager as BaseProblemManager } from "@theia/markers/lib/browser";
import { Diagnostic } from "vscode-languageserver-types";
import { Marker } from "@theia/markers/lib/common/marker";

import URI from "@elysium/core/lib/common/uri";

@injectable()
export class ProblemManager extends BaseProblemManager {
    public setMarkers(uri: URI, owner: string, data: Diagnostic[]): Marker<Diagnostic>[] {
        const uriString = uri.toString();
        const collection = this.uri2MarkerCollection.get(uriString) || new MarkerCollection<Diagnostic>(uri, this.getKind());
        const oldMarkers = collection.setMarkers(owner, data);
        const foundMarkers = this.findMarkers({ uri });

        if (data.length + foundMarkers.length > 0) this.uri2MarkerCollection.set(uriString, collection);
        else this.uri2MarkerCollection.delete(uriString);

        this.fireOnDidChangeMarkers(uri);

        return oldMarkers;
    }
}
