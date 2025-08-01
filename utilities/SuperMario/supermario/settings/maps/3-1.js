/* (c) https://github.com/MontiCore/monticore */
FullScreenMario.FullScreenMario.settings.maps.library["3-1"] = {
    "name": "3-1",
    "time": 300,
    "locations": [
        { "entry": "Plain" },
        { "entry": "PipeVertical" },
        { "xloc": 1272 },
        { "area": 1 },
        { "area": 2, "entry": "Vine" }
    ],
    "areas": [
        {
            "setting": "Overworld Night Alt",
            "blockBoundaries": true,
            "creation": [
                { "macro": "Floor", "width": 360 },
                { "macro": "CastleLarge", "x": -16 },
                { "macro": "Pattern", "pattern": "BackFence", "repeat": 5 },
                { "thing": "Block", "x": 128, "y": 32 },
                { "thing": "Block", "x": 152, "y": 40 },
                { "thing": "Block", "x": 176, "y": 40, "contents": "Mushroom" },
                { "thing": "Koopa", "x": 200, "y": 12, "jumping": true },
                { "macro": "Fill", "thing": "Brick", "x": 208, "y": 32, "xnum": 3 },
                { "thing": "Koopa", "x": 224, "y": 20, "jumping": true },
                { "macro": "Pipe", "x": 256, "height": 24, "piranha": true },
                { "thing": "Goomba", "x": 296, "y": 8 },
                { "macro": "Pipe", "x": 304, "height": 32, "piranha": true, "transport": 3 },
                { "macro": "Floor", "x": 384, "width": 232 },
                { "macro": "Fill", "thing": "Goomba", "x": 424, "y": 8, "xnum": 3, "xwidth": 12 },
                { "macro": "Pipe", "x": 456, "height": 24, "piranha": true },
                { "thing": "Brick", "x": 488, "y": 32 },
                { "thing": "Koopa", "x": 520, "y": 12 },
                { "macro": "Pipe", "x": 536, "height": 16, "piranha": true, "entrance": 1 },
                { "thing": "Stone", "x": 584, "y": 8 },
                { "thing": "Stone", "x": 592, "y": 16, "height": 16 },
                { "thing": "Stone", "x": 600, "y": 24, "height": 24 },
                { "thing": "Stone", "x": 608, "y": 32, "height": 32 },
                { "macro": "Water", "x": 616, "y": 10, "width": 64 },
                { "macro": "Bridge", "x": 616, "y": 32, "width": 64 },
                { "macro": "Fill", "thing": "Goomba", "x": 656, "y": 40, "xnum": 3, "xwidth": 12 },
                { "thing": "Block", "x": 656, "y": 64, "contents": "Mushroom1Up", "hidden": true },
                { "macro": "Floor", "x": 680 },
                { "thing": "Stone", "x": 680, "y": 32, "height": 32 },
                { "macro": "Water", "x": 688, "y": 10, "width": 16 },
                { "macro": "Floor", "x": 704, "width": 320 },
                { "thing": "Stone", "x": 704, "y": 32, "height": 32 },
                { "thing": "Stone", "x": 712, "y": 16, "height": 16 },
                { "thing": "Brick", "x": 720, "y": 64, "contents": "Star" },
                { "macro": "Fill", "thing": "Brick", "x": 728, "y": 64, "xnum": 2 },
                { "macro": "Fill", "thing": "Goomba", "x": 752, "y": 8, "xnum": 2, "xwidth": 12 },
                { "thing": "Koopa", "x": 808, "y": 12 },
                { "macro": "Pipe", "x": 824, "height": 32, "piranha": true },
                { "macro": "Fill", "thing": "Brick", "x": 888, "y": 32, "xnum": 11 },
                { "macro": "Fill", "thing": "Brick", "x": 888, "y": 64, "xnum": 2 },
                { "thing": "HammerBro", "x": 904, "y": 44 },
                { "thing": "Block", "x": 904, "y": 64 },
                { "macro": "Fill", "thing": "Brick", "x": 912, "y": 64, "xnum": 3 },
                { "thing": "HammerBro", "x": 936, "y": 12 },
                { "thing": "Block", "x": 936, "y": 64, "contents": "Mushroom" },
                { "macro": "Fill", "thing": "Brick", "x": 944, "y": 64, "xnum": 3 },
                // { "thing": "Springboard", "x": 1008, "y": 14.5 },
                { "macro": "Fill", "thing": "Brick", "x": 1032, "y": 40, "xnum": 3 },
                { "macro": "Fill", "thing": "Brick", "x": 1032, "y": 64, "xnum": 2 },
                { "thing": "Brick", "thing": "Brick", "x": 1048, "y": 64, "contents": ["Vine", { "entrance": 4 }] },
                { "macro": "Floor", "x": 1056, "width": 80 },
                { "thing": "Stone", "x": 1088, "y": 8 },
                { "thing": "Stone", "x": 1096, "y": 16, "height": 16 },
                { "thing": "Stone", "x": 1104, "y": 24, "height": 24 },
                { "thing": "Stone", "x": 1112, "y": 32, "height": 32 },
                { "thing": "Goomba", "x": 1112, "y": 40 },
                { "thing": "Stone", "x": 1120, "y": 40, "height": 40 },
                { "thing": "Goomba", "x": 1120, "y": 48 },
                { "thing": "Stone", "x": 1128, "y": 48, "height": 48 },
                { "macro": "Floor", "x": 1152, "width": 264 },
                { "thing": "Koopa", "x": 1192, "y": 12 },
                { "macro": "Fill", "thing": "Brick", "x": 1200, "y": 32, "xnum": 2, "ynum": 2, "xwidth": 16, "yheight": 32 },
                { "macro": "Fill", "thing": "Block", "x": 1208, "y": 32, "ynum": 2, "yheight": 32 },
                { "thing": "Koopa", "x": 1216, "y": 76 },
                { "macro": "Fill", "thing": "Goomba", "x": 1232, "y": 8, "xnum": 3, "xwidth": 12 },
                { "macro": "Fill", "thing": "Brick", "x": 1240, "y": 32, "xnum": 2, "ynum": 2, "xwidth": 16, "yheight": 32 },
                { "thing": "Block", "x": 1248, "y": 32, "contents": "Mushroom" },
                { "thing": "Block", "x": 1248, "y": 64 },
                { "thing": "Koopa", "x": 1320, "y": 12, "jumping": true },
                { "thing": "Brick", "x": 1328, "y": 32 },
                { "thing": "Brick", "x": 1336, "y": 32, "contents": "Coin" },
                { "thing": "Koopa", "x": 1344, "y": 18, "jumping": true },
                { "macro": "Fill", "thing": "Brick", "x": 1344, "y": 32, "xnum": 3 },
                { "thing": "Koopa", "x": 1360, "y": 44 },
                { "thing": "Koopa", "x": 1368, "y": 12, "jumping": true },
                { "thing": "Stone", "x": 1392, "y": 24, "height": 24 },
                { "thing": "Stone", "x": 1400, "y": 48, "height": 48 },
                { "macro": "Floor", "x": 1440, "width": 320 },
                { "thing": "Stone", "x": 1464, "y": 8 },
                { "thing": "Stone", "x": 1472, "y": 16, "height": 16 },
                { "thing": "Stone", "x": 1480, "y": 24, "height": 24 },
                { "thing": "Stone", "x": 1488, "y": 32, "height": 32 },
                { "thing": "Stone", "x": 1496, "y": 40, "height": 40 },
                { "thing": "Stone", "x": 1504, "y": 48, "height": 48 },
                { "thing": "Koopa", "x": 1504, "y": 60 },
                { "thing": "Stone", "x": 1512, "y": 56, "height": 56 },
                { "thing": "Stone", "x": 1520, "y": 64, "width": 16, "height": 64 },
                { "thing": "Koopa", "x": 1528, "y": 76 },
                { "macro": "EndOutsideCastle", "x": 1600, "transport": { "map": "3-2" } }
            ]
        }, {
            "setting": "Underworld",
            "blockBoundaries": true,
            "creation": [
                { "macro": "Floor", "width": 136 },
                { "macro": "Fill", "thing": "Brick", "y": 8, "ynum": 11 },
                { "macro": "Fill", "thing": "Brick", "x": 24, "y": 40, "xnum": 2, "ynum": 4, "xwidth": 72 },
                { "macro": "Fill", "thing": "Brick", "x": 32, "y": 32, "xnum": 2, "xwidth": 56 },
                { "macro": "Fill", "thing": "Brick", "x": 32, "y": 56, "xnum": 2, "ynum": 2, "xwidth": 56 },
                { "macro": "Fill", "thing": "Coin", "x": 33, "y": 39, "xnum": 2, "xwidth": 56 },
                { "macro": "Fill", "thing": "Brick", "x": 40, "y": 40, "xnum": 2, "xwidth": 40 },
                { "thing": "Brick", "x": 40, "y": 64, "contents": "Mushroom" },
                { "macro": "Fill", "thing": "Coin", "x": 41, "y": 47, "xnum": 2, "xwidth": 40 },
                { "macro": "Fill", "thing": "Brick", "x": 48, "y": 48, "xnum": 2, "xwidth": 24 },
                { "macro": "Fill", "thing": "Coin", "x": 49, "y": 55, "xnum": 2, "ynum": 2, "xwidth": 24, "yheight": 16 },
                { "macro": "Fill", "thing": "Brick", "x": 56, "y": 56, "xnum": 2, "ynum": 2 },
                { "macro": "Fill", "thing": "Coin", "x": 57, "y": 71, "xnum": 2, "ynum": 2, "xwidth": 8, "yheight": 8 },
                { "thing": "Brick", "x": 80, "y": 64 },
                { "thing": "PipeHorizontal", "x": 104, "y": 16, "entrance": 1 },
                { "thing": "PipeVertical", "x": 120, "y": 88, "height": 88 }
            ]
        }, {
            "setting": "Sky Night",
            "blockBoundaries": false,
            "exit": 2,
            "creation": [
                { "thing": "Stone", "width": 32 },
                { "thing": "Stone", "x": 40, "width": 624 },
                { "thing": "Platform", "x": 128, "y": 24, "width": 24, "transport": true },
                { "macro": "Fill", "thing": "Coin", "x": 121, "y": 55, "xnum": 16, "xwidth": 8 },
                { "thing": "Stone", "x": 256, "y": 40 },
                { "macro": "Fill", "thing": "Coin", "x": 273, "y": 55, "xnum": 16, "xwidth": 8 },
                { "thing": "Stone", "x": 408, "y": 48, "height": 16 },
                { "macro": "Fill", "thing": "Coin", "x": 425, "y": 63, "xnum": 7, "xwidth": 8 },
                { "thing": "Stone", "x": 488, "y": 48, "height": 16 },
                { "thing": "Stone", "x": 536, "y": 56, "width": 16 },
                { "macro": "Fill", "thing": "Stone", "x": 568, "y": 56, "xnum": 5, "xwidth": 16 },
                { "macro": "Fill", "thing": "Coin", "x": 569, "y": 63, "xnum": 10, "xwidth": 8 },
                { "macro": "Fill", "thing": "Coin", "x": 681, "y": 15, "xnum": 3, "xwidth": 8 }
            ]
        }
    ]
};
