/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { EMBEDDEDMONTIARCMATH_LANGUAGE_ID, EMBEDDEDMONTIARCMATH_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./embeddedmontiarcmath-monaco-language";

monaco.languages.register({
    id: EMBEDDEDMONTIARCMATH_LANGUAGE_ID,
    extensions: [".emam"],
    aliases: [EMBEDDEDMONTIARCMATH_LANGUAGE_NAME],
    mimetypes: ["text/x-embeddedmontiarcmath"]
});

monaco.languages.onLanguage(EMBEDDEDMONTIARCMATH_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(EMBEDDEDMONTIARCMATH_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(EMBEDDEDMONTIARCMATH_LANGUAGE_ID, monarchLanguage);
});
