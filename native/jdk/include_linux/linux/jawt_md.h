/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
#ifndef _JAVASOFT_JAWT_MD_H_
#define _JAVASOFT_JAWT_MD_H_

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Intrinsic.h>
#include "jawt.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * X11-specific declarations for AWT native interface.
 * See notes in jawt.h for an example of use.
 */
typedef struct jawt_X11DrawingSurfaceInfo {
    Drawable drawable;
    Display* display;
    VisualID visualID;
    Colormap colormapID;
    int depth;
    /*
     * Since 1.4
     * Returns a pixel value from a set of RGB values.
     * This is useful for paletted color (256 color) modes.
     */
    int (JNICALL *GetAWTColor)(JAWT_DrawingSurface* ds,
        int r, int g, int b);
} JAWT_X11DrawingSurfaceInfo;

#ifdef __cplusplus
}
#endif

#endif /* !_JAVASOFT_JAWT_MD_H_ */
