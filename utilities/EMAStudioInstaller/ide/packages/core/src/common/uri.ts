/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import URIBase from "@theia/core/lib/common/uri";
import Uri from "vscode-uri";

export default class URI extends URIBase {
    public constructor(uri?: string | Uri) {
        super(uri);
    }

    /**
     * Return a new uri with a query formed from the given query params map.
     */
    public setQueryParams(params: Map<string, string>): URI {
        let query = '';

        params.forEach((value, key) => {
            query += `&${key}=${value}`;
        });

        query = query.slice(1);

        return this.withQuery(query) as URI;
    }

    /**
     * Return a Map with the query params and their respective keys
     */
    public getQueryParams(): Map<string, string> {
        const queryParams = new Map();

        this.query.split('&').forEach(params => {
            const values = params.split('=');

            queryParams.set(values[0], values[1]);
        });

        return queryParams;
    }

    /**
     * Return whether the key is in the query params
     */
    public hasQueryParam(key: string): boolean {
        return this.getQueryParams().has(key);
    }

    /**
     * Return the value of a key in the query params
     */
    public getQueryParam(key: string): string | undefined {
        return this.getQueryParams().get(key);
    }
}
