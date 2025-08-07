/* (c) https://github.com/MontiCore/monticore */
export class Utilities {
    public static query(queryString: string): any {
        const pairs = queryString.split('&');
        const result = {};

        for(let pair of pairs) {
            const parts = pair.split('=');
            const key = parts[0];

            result[key] = parts[1];
        }

        return result;
    }

    public static occurrences(string: string, search: string): number {
        let occurrences = 0;
        let position = 0;

        while(true) {
            position = string.indexOf(search, position);

            if(position >= 0) {
                occurrences++;
                position += search.length;
            } else break;
        }

        return occurrences;
    }
}
