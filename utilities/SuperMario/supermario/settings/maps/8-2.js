/* (c) https://github.com/MontiCore/monticore */
FullScreenMario.FullScreenMario.settings.maps.library["8-2"] = {
    "name": "8-2",
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
                { "macro": "Pattern", "pattern": "BackFenceMin2", "repeat": 4 },
                { "macro": "CastleSmall" },
                { "macro": "Floor", "width": 120 },
                { "macro": "Floor", "x": 128, "width": 40 },
                { "thing": "Lakitu", "x": 128, "y": 84 },
                { "thing": "Stone", "x": 136, "y": 8 },
                { "thing": "Stone", "x": 144, "y": 16, "height": 16 },
                { "thing": "Stone", "x": 152, "y": 24, "height": 24 },
                { "thing": "Stone", "x": 160, "y": 32, "height": 32 },
                { "macro": "Floor", "x": 176, "width": 112 },
                { "thing": "Stone", "x": 176, "y": 48, "height": 48 },
                { "thing": "Stone", "x": 184, "y": 56, "height": 56 },
                { "thing": "Stone", "x": 192, "y": 64, "width": 16, "height": 64 },
                { "macro": "Fill", "thing": "Block", "x": 232, "y": 32, "xnum": 4 },
                { "macro": "Floor", "x": 296, "width": 64 },
                { "thing": "Brick", "x": 344, "y": 64 },
                { "thing": "Springboard", "x": 352, "y": 14.5 },
                { "thing": "Brick", "x": 352, "y": 64, "contents": "Mushroom1Up" },
                { "macro": "Fill", "thing": "Brick", "x": 360, "y": 64, "xnum": 31 },
                { "macro": "Floor", "x": 368, "width": 32 },
                { "macro": "Floor", "x": 408 },
                { "macro": "Floor", "x": 424, "width": 24 },
                { "macro": "Floor", "x": 456, "width": 48 },
                { "thing": "Koopa", "x": 456, "y": 26, "jumping": true },
                { "macro": "Floor", "x": 512, "width": 112 },
                { "macro": "Fill", "thing": "Brick", "x": 616, "y": 32, "xnum": 2 },
                { "thing": "PlantLarge", "x": 552, "y": 23 },
                { "macro": "Floor", "x": 640, "width": 32 },
                { "macro": "Floor", "x": 680, "width": 424 },
                { "thing": "Cannon", "x": 680, "y": 16, "height": 16 },
                { "thing": "Koopa", "x": 736, "y": 32, "jumping": true },
                { "thing": "Cannon", "x": 744, "y": 8, "nofire": true },
                { "thing": "Koopa", "x": 760, "y": 24, "jumping": true },
                { "thing": "Brick", "x": 792, "y": 32 },
                { "thing": "Brick", "x": 800, "y": 32, "contents": "Mushroom" },
                { "thing": "Cannon", "x": 840, "y": 16, "height": 16 },
                { "macro": "Fill", "thing": "Brick", "x": 880, "y": 32, "xnum": 8 },
                { "thing": "Fence", "x": 888, "y": 8 },
                { "thing": "Beetle", "x": 888, "y": 8.5 },
                { "thing": "Cannon", "x": 920, "y": 8 },
                { "thing": "Brick", "x": 944, "y": 32 },
                { "thing": "PlantLarge", "x": 936, "y": 23 },
                { "thing": "Stone", "x": 952, "y": 32 },
                { "thing": "Cannon", "x": 952, "y": 40 },
                { "thing": "Brick", "x": 960, "y": 32, "contents": "Mushroom" },
                { "thing": "Beetle", "x": 968, "y": 8.5 },
                { "thing": "Beetle", "x": 984, "y": 8.5 },
                { "thing": "Cannon", "x": 1000, "y": 24, "height": 24 },
                { "macro": "Pipe", "x": 1048, "height": 16, "piranha": true },
                { "macro": "Floor", "x": 1112, "width": 40 },
                { "thing": "Koopa", "x": 1112, "y": 12, "jumping": true },
                { "macro": "Pipe", "x": 1136, "height": 16, "piranha": true },
                { "macro": "Floor", "x": 1160 },
                { "macro": "Floor", "x": 1176 },
                { "macro": "Floor", "x": 1232, "width": 160 },
                { "thing": "Fence", "x": 1272, "y": 8 },
                { "macro": "Pipe", "x": 1248, "height": 32, "piranha": true, "transport": 2 },
                { "macro": "Pipe", "x": 1304, "height": 16, "piranha": true, "entrance": 1 },
                { "thing": "PlantLarge", "x": 1320, "y": 23 },
                { "thing": "Koopa", "x": 1360, "y": 32, "jumping": true },
                { "thing": "Koopa", "x": 1376, "y": 24, "jumping": true },
                { "macro": "Floor", "x": 1400 },
                { "thing": "Cannon", "x": 1400, "y": 16, "height": 16 },
                { "thing": "Koopa", "x": 1400, "y": 48, "jumping": true },
                { "macro": "Floor", "x": 1432, "width": 184 },
                { "thing": "Stone", "x": 1456, "y": 8 },
                { "thing": "Stone", "x": 1464, "y": 16, "height": 16 },
                { "thing": "Stone", "x": 1472, "y": 24, "height": 24 },
                { "thing": "Goomba", "x": 1472, "y": 32 },
                { "thing": "Stone", "x": 1480, "y": 32, "height": 32 },
                { "thing": "Stone", "x": 1488, "y": 40, "height": 40 },
                { "thing": "Goomba", "x": 1488, "y": 48 },
                { "thing": "Beetle", "x": 1512, "y": 8.5 },
                { "thing": "Cannon", "x": 1528, "y": 8, "nofire": true },
                { "thing": "Cannon", "x": 1528, "y": 24, "height": 16 },
                { "macro": "Pattern", "pattern": "BackFenceMin2", "x": 1536, "skips": [2, 7] },
                { "thing": "Stone", "x": 1592, "y": 8 },
                { "thing": "Stone", "x": 1600, "y": 16, "height": 16 },
                { "thing": "Stone", "x": 1608, "y": 24, "height": 24 },
                { "macro": "Floor", "x": 1624 },
                { "thing": "Stone", "x": 1624, "y": 40, "height": 40 },
                { "thing": "Koopa", "x": 1624, "y": 72, "jumping": true },
                { "macro": "Floor", "x": 1648, "width": 320 },
                { "macro": "LakituStop", "x": 1648 },
                { "thing": "Stone", "x": 1648, "y": 64, "width": 16, "height": 64 },
                { "thing": "PlantLarge", "x": 1704, "y": 23 },
                { "macro": "EndOutsideCastle", "x": 1728, "transport": { "map": "8-3" } }
            ]
        }, {
            "setting": "Underworld",
            "blockBoundaries": true,
            "creation": [
                { "macro": "Ceiling", "x": 32, "width": 7 },
                { "macro": "Floor", "width": 136 },
                { "macro": "Fill", "thing": "Brick", "y": 8, "ynum": 11 },
                { "macro": "Fill", "thing": "Brick", "x": 32, "y": 48, "xnum": 7 },
                { "thing": "Brick", "x": 32, "y": 56 },
                { "macro": "Fill", "thing": "Coin", "x": 42, "y": 55, "xnum": 5, "ynum": 2, "xwidth": 8, "yheight": 8 },
                { "macro": "Fill", "thing": "Brick", "x": 80, "y": 56, "ynum": 4 },
                { "macro": "Fill", "thing": "Brick", "x": 88, "y": 56, "xnum": 2 },
                { "thing": "Brick", "x": 112, "y": 48, "contents": "Coin" },
                { "thing": "PipeHorizontal", "x": 104, "y": 16, "transport": 1 },
                { "thing": "PipeVertical", "x": 120, "y": 88, "height": 88 }
            ]
        }
    ]
};
