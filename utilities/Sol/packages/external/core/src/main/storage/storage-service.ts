/*
 * (c) https://github.com/MontiCore/monticore
 */
import { inject, injectable } from "inversify";
import { Application } from "../application";

import * as os from "os";
import * as path from "path";
import * as fs from "fs-extra";

export const StorageService = Symbol("StorageService");
/**
 * An interface which should be implemented by classes which implement the necessary functionality to store
 * key value pairs.
 */
export interface StorageService {
    /**
     * Stores given data under a given key.
     * @param key The key under which the data should be stored.
     * @param data The data to be stored.
     */
    setData<T>(key: string, data: T): Promise<void>;

    /**
     * Fetches data for a given key if present or returns the default value if specified.
     * @param key The key for which the data should be fetched.
     * @param defaultValue The default value which should be returned if no data is stored under the given key.
     * @return The data stored under the given key.
     */
    getData<T>(key: string, defaultValue?: T): Promise<T | undefined>;
}

@injectable()
export class StorageServiceImpl implements StorageService {
    @inject(Application) protected readonly application: Application;

    public async setData<T>(key: string, data: T): Promise<void> {
        const filePath = this.getFilePath(key);

        await fs.ensureFile(filePath);
        return fs.writeJSON(filePath, data);
    }

    public async getData<T>(key: string, defaultValue?: T): Promise<T | undefined> {
        const filePath = this.getFilePath(key);

        if (await fs.pathExists(filePath)) return fs.readJSON(this.getFilePath(key));
        else return defaultValue;
    }

    protected getFilePath(key: string): string {
        return path.join(os.homedir(), `.${this.application.getName().toLowerCase()}`, `${key}.json`);
    }
}
