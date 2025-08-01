/* (c) https://github.com/MontiCore/monticore */
FullScreenMario.FullScreenMario.settings.maps.library["8-1"] = {
    "name": "8-1",
    "time": 300,
    "locations": [
        { "entry": "Plain" },
        { "entry": "PipeVertical" },
        { "area": 1 }
    ],
    "areas": [
        {
            "setting": "Overworld",
            "blockBoundaries": true,
            "creation": [
                { "macro": "Pattern", "pattern": "BackFence", "repeat": 7 },
                { "macro": "CastleLarge", "x": -16 },
                { "macro": "Floor", "width": 368 },
                { "thing": "Beetle", "x": 144, "y": 8.5 },
                { "macro": "Fill", "thing": "Goomba", "x": 184, "y": 8, "xnum": 3, "xwidth": 12 },
                { "macro": "Fill", "thing": "Goomba", "x": 240, "y": 8, "xnum": 3, "xwidth": 12 },
                { "macro": "Pipe", "x": 280, "height": 32, "piranha": true },
                { "macro": "Fill", "thing": "Koopa", "x": 344, "y": 12, "xnum": 2, "xwidth": 12 },
                { "macro": "Floor", "x": 376 },
                { "macro": "Floor", "x": 392, "width": 16 },
                { "macro": "Floor", "x": 416, "width": 16 },
                { "macro": "Floor", "x": 440, "width": 16 },
                { "macro": "Floor", "x": 464, "width": 888 },
                { "thing": "Koopa", "x": 488, "y": 12 },
                { "thing": "Coin", "x": 513, "y": 39 },
                { "macro": "Fill", "thing": "Goomba", "x": 552, "y": 8, "xnum": 3, "xwidth": 12 },
                { "macro": "Pipe", "x": 608, "height": 32, "piranha": true },
                { "thing": "Block", "x": 640, "y": 40, "contents": "Mushroom1Up", "hidden": true },
                { "thing": "Beetle", "x": 648, "y": 8.5 },
                { "macro": "Pipe", "x": 656, "height": 24, "piranha": true },
                { "thing": "Coin", "x": 713, "y": 39 },
                { "macro": "Pipe", "x": 752, "height": 32, "piranha": true },
                { "thing": "Coin", "x": 786, "y": 39 },
                { "macro": "Pipe", "x": 832, "height": 32, "piranha": true, "transport": 2 },
                { "macro": "Fill", "thing": "Goomba", "x": 864, "y": 8, "xnum": 3, "xwidth": 12 },
                { "macro": "Fill", "thing": "Coin", "x": 873, "y": 71, "xnum": 2, "xwidth": 8 },
                { "macro": "Pipe", "x": 920, "height": 16, "piranha": true, "entrance": 1 },
                { "thing": "Koopa", "x": 952, "y": 12 },
                { "macro": "Fill", "thing": "Koopa", "x": 992, "y": 12, "xnum": 3, "xwidth": 12 },
                { "macro": "Fill", "thing": "Koopa", "x": 1040, "y": 12, "xnum": 3, "xwidth": 12 },
                { "macro": "Pipe", "x": 1120, "height": 24, "piranha": true },
                { "macro": "Fill", "thing": "Goomba", "x": 1184, "y": 8, "xnum": 3, "xwidth": 12 },
                { "thing": "Stone", "x": 1224, "y": 32, "height": 32 },
                { "macro": "Fill", "thing": "Brick", "x": 1232, "y": 64, "xnum": 4 },
                { "thing": "Block", "x": 1264, "y": 32, "hidden": true },
                { "thing": "Brick", "x": 1264, "y": 64, "contents": "Mushroom" },
                { "macro": "Fill", "thing": "Brick", "x": 1272, "y": 64, "xnum": 3 },
                { "thing": "Koopa", "x": 1288, "y": 32, "jumping": true },
                { "thing": "Stone", "x": 1304, "y": 32, "height": 32 },
                { "macro": "Floor", "x": 1360 },
                { "macro": "Floor", "x": 1376, "width": 16 },
                { "thing": "Koopa", "x": 1376, "y": 32, "jumping": true },
                { "macro": "Floor", "x": 1400 },
                { "macro": "Floor", "x": 1416, "width": 16 },
                { "thing": "Koopa", "x": 1416, "y": 28, "jumping": true },
                { "macro": "Floor", "x": 1416, "width": 16 },
                { "macro": "Floor", "x": 1440, "width": 136 },
                { "macro": "Fill", "thing": "Brick", "x": 1472, "y": 40, "xnum": 2 },
                { "thing": "Brick", "x": 1488, "y": 40, "contents": "Star" },
                { "macro": "Fill", "thing": "Brick", "x": 1496, "y": 40, "xnum": 5 },
                { "macro": "Floor", "x": 1584 },
                { "macro": "Floor", "x": 1600 },
                { "macro": "Floor", "x": 1616, "width": 152 },
                { "macro": "Fill", "thing": "Koopa", "x": 1656, "y": 12, "xnum": 2, "xwidth": 12 },
                { "thing": "Stone", "x": 1680, "y": 16, "height": 16 },
                { "macro": "Fill", "thing": "Coin", "x": 1785, "y": 39, "xnum": 2, "xwidth": 8 },
                { "macro": "Floor", "x": 1816, "width": 80 },
                { "macro": "Fill", "thing": "Goomba", "x": 1856, "y": 8, "xnum": 3, "xwidth": 12 },
                { "macro": "Floor", "x": 1904, "width": 16 },
                { "macro": "Pipe", "x": 1904, "height": 24, "piranha": true },
                { "macro": "Floor", "x": 1936, "height": 32, "piranha": true },
                { "macro": "Floor", "x": 1968, "width": 352 },
                { "macro": "Pipe", "x": 1968, "height": 40 },
                { "thing": "Beetle", "x": 2032, "y": 8.5 },
                { "macro": "Fill", "thing": "Goomba", "x": 2056, "y": 8, "xnum": 3, "xwidth": 12 },
                { "macro": "Fill", "thing": "Goomba", "x": 2112, "y": 8, "xnum": 3, "xwidth": 12 },
                { "macro": "Fill", "thing": "Goomba", "x": 2176, "y": 8, "xnum": 2, "xwidth": 12 },
                { "thing": "Stone", "x": 2200, "y": 8 },
                { "thing": "Stone", "x": 2208, "y": 16, "height": 16 },
                { "thing": "Stone", "x": 2216, "y": 24, "height": 24 },
                { "thing": "Stone", "x": 2224, "y": 32, "height": 32 },
                { "thing": "Stone", "x": 2232, "y": 40, "height": 40 },
                { "thing": "Stone", "x": 2240, "y": 48, "height": 48 },
                { "thing": "Beetle", "x": 2264, "y": 8.5 },
                { "macro": "Fill", "thing": "Coin", "x": 2265, "y": 39, "xnum": 2, "xwidth": 8 },
                { "macro": "Fill", "thing": "Coin", "x": 2329, "y": 39, "xnum": 2, "xwidth": 40 },
                { "macro": "Floor", "x": 2344, "width": 16 },
                { "macro": "Floor", "x": 2384, "width": 128 },
                { "macro": "Fill", "thing": "Stone", "x": 2424, "y": 16, "xnum": 2, "xwidth": 32, "height": 16 },
                { "thing": "Koopa", "x": 2440, "y": 12 },
                { "macro": "Fill", "thing": "Coin", "x": 2529, "y": 39, "xnum": 2, "xwidth": 8 },
                { "macro": "Floor", "x": 2552 },
                { "macro": "Fill", "thing": "Coin", "x": 2569, "y": 39, "xnum": 2, "xwidth": 8 },
                { "macro": "Floor", "x": 2600, "width": 272 },
                { "thing": "Koopa", "x": 2656, "y": 12 },
                { "macro": "Pattern", "pattern": "BackFence", "x": 2688, "skips": [5, 10] },
                { "macro": "Fill", "thing": "Koopa", "x": 2712, "y": 12, "xnum": 3, "xwidth": 12 },
                { "macro": "Pipe", "x": 2752, "height": 24, "piranha": true },
                { "macro": "Pipe", "x": 2840, "height": 16, "piranha": true },
                { "macro": "Floor", "x": 2880 },
                { "thing": "Stone", "x": 2880, "y": 16, "height": 16 },
                { "macro": "Floor", "x": 2896 },
                { "thing": "Stone", "x": 2896, "y": 32, "height": 32 },
                { "macro": "Floor", "x": 2912 },
                { "thing": "Stone", "x": 2912, "y": 48, "height": 48 },
                { "macro": "Floor", "x": 2928, "width": 288 },
                { "thing": "Stone", "x": 2928, "y": 64, "width": 16, "height": 64 },
                { "macro": "EndOutsideCastle", "x": 3008, "transport": { "map": "8-2" } },
                { "macro": "Pattern", "pattern": "BackFence", "x": 3072, "skips": [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] }
            ]
        }, {
            "setting": "Underworld",
            "blockBoundaries": true,
            "creation": [
                { "macro": "Floor", "width": 136 },
                { "macro": "Fill", "thing": "Brick", "y": 8, "ynum": 11 },
                { "macro": "Fill", "thing": "Brick", "x": 24, "y": 32, "xnum": 9 },
                { "macro": "Fill", "thing": "Brick", "x": 24, "y": 64, "xnum": 10, "ynum": 4 },
                { "macro": "Fill", "thing": "Coin", "x": 25, "y": 7, "xnum": 9, "xwidth": 8 },
                { "macro": "Fill", "thing": "Coin", "x": 33, "y": 39, "xnum": 8, "xwidth": 8 },
                { "thing": "Brick", "x": 96, "y": 32, "contents": "Coin" },
                { "macro": "Fill", "thing": "Brick", "x": 104, "y": 24, "xnum": 2, "ynum": 9 },
                { "thing": "PipeHorizontal", "x": 104, "y": 16, "transport": 1 },
                { "thing": "PipeVertical", "x": 120, "y": 100, "height": 100 }
            ]
        }
    ]
};
