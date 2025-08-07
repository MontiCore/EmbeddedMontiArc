/* (c) https://github.com/MontiCore/monticore */
export type Platform = "all" | "win32" | "linux";

export namespace Platform {
    export function toFolderName(platform: Platform) {
        switch (platform) {
            case "win32": return "windows";
            case "linux": return "linux";
            default: return "common";
        }
    }
}
