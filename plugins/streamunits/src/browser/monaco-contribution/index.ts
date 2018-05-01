/*
 * Copyright (C) 2018 SE RWTH.
 *
 * TODO: Include License.
 */

import { STREAMUNITS_LANGUAGE_ID, STREAMUNITS_LANGUAGE_NAME } from "../../common";
import { configuration, monarchLanguage } from "./streamunits-monaco-language";

monaco.languages.register({
    id: STREAMUNITS_LANGUAGE_ID,
    extensions: [".stream"],
    aliases: [STREAMUNITS_LANGUAGE_NAME],
    mimetypes: ["text/x-streamunits"]
});

monaco.languages.onLanguage(STREAMUNITS_LANGUAGE_ID, () => {
    monaco.languages.setLanguageConfiguration(STREAMUNITS_LANGUAGE_ID, configuration);
    monaco.languages.setMonarchTokensProvider(STREAMUNITS_LANGUAGE_ID, monarchLanguage);
});
