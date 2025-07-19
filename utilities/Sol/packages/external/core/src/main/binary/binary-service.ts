/*
 * (c) https://github.com/MontiCore/monticore
 */
import { injectable } from "inversify";
import { Deferred } from "ts-deferred";

import * as hasbin from "hasbin";

export const BinaryService = Symbol("BinaryService");
/**
 * An interface to be implemented by classes which hold the functionality to check the PATH environmental variable
 * for the existence of one or more binaries.
 */
export interface BinaryService {
    /**
     * Checks whether a given binary is available in the PATH environmental variable.
     * @param binary The binary to be checked against.
     * @return True if the binary is available, false otherwise.
     */
    has(binary: string): Promise<boolean>;

    /**
     * Checks whether all given binaries are available in the PATH environmental variable.
     * @param binaries The binaries to be checked against.
     * @return True if the binaries are available, false otherwise.
     */
    hasAll(binaries: string[]): Promise<boolean>;

    /**
     * Checks whether some of the given binaries are available in the PATH environmental variable.
     * @param binaries The binaries to be checked against.
     * @return True if some of the binaries are available, false otherwise.
     */
    hasSome(binaries: string[]): Promise<boolean>;

    /**
     * Fetches the first binary it encounters in the PATH environmental variable amongst given binaries.
     * @param binaries The binaries acting as search values.
     * @return The name of the first binary available.
     */
    getFirst(binaries: string[]): Promise<string | boolean>;
}

@injectable()
export class BinaryServiceImpl implements BinaryService {
    public async has(binary: string): Promise<boolean> {
        const deferred = new Deferred<boolean>();

        hasbin(binary, deferred.resolve);

        return deferred.promise;
    }

    public async hasAll(binaries: string[]): Promise<boolean> {
        const deferred = new Deferred<boolean>();

        hasbin.all(binaries, deferred.resolve);

        return deferred.promise;
    }

    public async hasSome(binaries: string[]): Promise<boolean> {
        const deferred = new Deferred<boolean>();

        hasbin.some(binaries, deferred.resolve);

        return deferred.promise;
    }

    public async getFirst(binaries: string[]): Promise<string | boolean> {
        const deferred = new Deferred<string | boolean>();

        hasbin.first(binaries, deferred.resolve);

        return deferred.promise;
    }
}
