/* (c) https://github.com/MontiCore/monticore */
import * as Util from "util";

export class Constants {
    public static readonly HOST = process.env.HOST ? process.env.HOST : Constants.getTravisHost();

    private static getTravisHost(): string {
        const slug = process.env.TRAVIS_REPO_SLUG;
        const parts = slug.split('/');

        return Util.format("https://%s.github.io/%s", parts[0], parts[1]);
    }

    public static readonly DEFAULT_TIMEOUT = 3000;
}
