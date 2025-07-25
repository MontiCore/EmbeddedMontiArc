/* (c) https://github.com/MontiCore/monticore */
FullScreenMario.FullScreenMario.settings.maps.library["4-3"] = {
    "name": "4-3",
    "time": 300,
    "locations": [
        { "entry": "Plain" }
    ],
    "areas": [
        {
            "setting": "Overworld Shrooms",
            "blockBoundaries": true,
            "creation": [
                { "macro": "Pattern", "pattern": "BackCloud", "x": 0, "y": 4, "repeat": 3 },
                { "macro": "Floor", "x": 0, "y": 0, "width": 120 },
                { "macro": "CastleSmall" },
                { "macro": "Shroom", "x": 128, "width": 40 },
                { "macro": "Shroom", "x": 152, "y": 64, "width": 40 },
                { "macro": "Fill", "thing": "Coin", "x": 161, "y": 71, "xnum": 3, "xwidth": 8 },
                { "macro": "Shroom", "x": 184, "y": 32, "width": 56 },
                { "macro": "Fill", "thing": "Coin", "x": 193, "y": 39, "xnum": 4, "xwidth": 8 },
                { "macro": "Fill", "thing": "Koopa", "x": 224, "y": 44, "xnum": 2, "smart": true },
                { "macro": "Shroom", "x": 256, "y": 72, "width": 24 },
                { "macro": "Shroom", "x": 288, "y": 8, "width": 56 },
                { "thing": "Koopa", "x": 288, "y": 84, "smart": true, "floating": true, "jumping": true, "begin": 32, "end": 88 },
                { "thing": "Coin", "x": 302, "y": 15 },
                { "thing": "Koopa", "x": 312, "y": 20, "smart": true },
                { "macro": "Shroom", "x": 312, "y": 64, "width": 40 },
                { "thing": "Coin", "x": 321, "y": 15 },
                { "thing": "Block", "x": 344, "y": 88, "contents": "Mushroom" },
                { "macro": "Shroom", "x": 352, "y": 32, "width": 24 },
                { "thing": "Coin", "x": 385, "y": 47 },
                { "macro": "Scale", "x": 396, "y": 86, "between": 56, "dropRight": 44 },
                { "macro": "Shroom", "x": 408, "y": 40, "width": 24 },
                { "thing": "Platform", "x": 464, "y": 20, "width": 24, "floating": true, "begin": 16, "end": 72 },
                { "thing": "Platform", "x": 496, "y": 66, "width": 24, "floating": true, "begin": 32, "end": 88 },
                { "macro": "Shroom", "x": 520, "width": 40 },
                { "macro": "Shroom", "x": 536, "y": 48, "width": 24 },
                { "macro": "Fill", "thing": "Coin", "x": 537, "y": 55, "xnum": 3, "xwidth": 8 },
                { "thing": "Koopa", "x": 544, "y": 12, "smart": true },
                { "macro": "Shroom", "x": 560, "y": 80, "width": 24 },
                { "macro": "Fill", "thing": "Coin", "x": 561, "y": 87, "xnum": 3, "xwidth": 8 },
                { "macro": "Shroom", "x": 576, "y": 32, "width": 24 },
                { "thing": "Coin", "x": 585, "y": 39 },
                { "macro": "Shroom", "x": 592, "y": 64, "width": 40 },
                { "thing": "Koopa", "x": 624, "y": 76, "smart": true },
                { "macro": "Scale", "x": 652, "y": 86, "between": 64, "dropRight": 52 },
                { "macro": "Shroom", "x": 672, "y": 32 },
                { "macro": "Scale", "x": 740, "y": 86, "dropRight": 56 },
                { "thing": "Coin", "x": 770, "y": 47 },
                { "macro": "Shroom", "x": 792, "y": 16, "width": 24 },
                { "macro": "Scale", "x": 828, "y": 86, "between": 48, "dropRight": 56 },
                { "macro": "Shroom", "x": 840, "y": 24, "width": 24 },
                { "macro": "Shroom", "x": 904, "y": 32, "width": 40 },
                { "macro": "Fill", "thing": "Coin", "x": 905, "y": 39, "xnum": 5, "xwidth": 8 },
                { "macro": "Shroom", "x": 936, "y": 56, "width": 24 },
                { "macro": "Shroom", "x": 968, "width": 56 },
                { "macro": "Shroom", "x": 1040, "y": 24, "width": 40 },
                { "thing": "Platform", "x": 1088, "y": 67, "width": 24, "floating": true, "begin": 8, "end": 88 },
                { "macro": "Floor", "x": 1128, "width": 152 },
                { "macro": "EndOutsideCastle", "x": 1176, "large": true, "walls": 3, "transport": { "map": "4-4" } }
            ]
        }
    ]
};
