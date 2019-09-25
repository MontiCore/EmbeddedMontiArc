/*
 * (c) https://github.com/MontiCore/monticore
 */
export const ExternalFileSystem = Symbol("ExternalFileSystem");
export interface ExternalFileSystem {
    getExternalUri(uri: string): Promise<string | undefined>;
    getCurrentUserHome(): Promise<string | undefined>;
}
