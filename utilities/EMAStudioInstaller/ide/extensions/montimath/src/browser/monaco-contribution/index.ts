/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { MONTIMATH_LANGUAGE_ID, MONTIMATH_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./montimath-monaco-language";

monaco.languages.register({
    id: MONTIMATH_LANGUAGE_ID,
    extensions: [".m"],
    aliases: [MONTIMATH_LANGUAGE_NAME],
    mimetypes: ["text/x-montimath"]
});

monaco.languages.onLanguage(MONTIMATH_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(MONTIMATH_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(MONTIMATH_LANGUAGE_ID, monarchLanguage);
});
