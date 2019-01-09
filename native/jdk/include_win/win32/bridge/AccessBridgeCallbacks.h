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
/*
 * AccessBridgeCallbacks.h      1.17 05/03/21
 */

/*
 * Header file defining callback typedefs for Windows routines
 * which are called from Java (responding to events, etc.).
 */

#ifndef __AccessBridgeCallbacks_H__
#define __AccessBridgeCallbacks_H__

#include <jni.h>
#include "AccessBridgePackages.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*AccessBridge_PropertyChangeFP) (long vmID, JOBJECT64 event, JOBJECT64 source,
                                               wchar_t *property, wchar_t *oldValue, wchar_t *newValue);

typedef void (*AccessBridge_JavaShutdownFP) (long vmID);
typedef void (*AccessBridge_JavaShutdownFP) (long vmID);

typedef void (*AccessBridge_FocusGainedFP) (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_FocusLostFP) (long vmID, JOBJECT64 event, JOBJECT64 source);

typedef void (*AccessBridge_CaretUpdateFP) (long vmID, JOBJECT64 event, JOBJECT64 source);

typedef void (*AccessBridge_MouseClickedFP) (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_MouseEnteredFP) (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_MouseExitedFP) (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_MousePressedFP) (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_MouseReleasedFP) (long vmID, JOBJECT64 event, JOBJECT64 source);

typedef void (*AccessBridge_MenuCanceledFP) (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_MenuDeselectedFP) (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_MenuSelectedFP) (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_PopupMenuCanceledFP) (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_PopupMenuWillBecomeInvisibleFP) (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_PopupMenuWillBecomeVisibleFP) (long vmID, JOBJECT64 event, JOBJECT64 source);

typedef void (*AccessBridge_PropertyNameChangeFP) (long vmID, JOBJECT64 event, JOBJECT64 source,
                                                                                               wchar_t *oldName, wchar_t *newName);
typedef void (*AccessBridge_PropertyDescriptionChangeFP) (long vmID, JOBJECT64 event, JOBJECT64 source,
                                                                                                      wchar_t *oldDescription, wchar_t *newDescription);
typedef void (*AccessBridge_PropertyStateChangeFP) (long vmID, JOBJECT64 event, JOBJECT64 source,
                                                                                                wchar_t *oldState, wchar_t *newState);
typedef void (*AccessBridge_PropertyValueChangeFP) (long vmID, JOBJECT64 event, JOBJECT64 source,
                                                                                                wchar_t *oldValue, wchar_t *newValue);
typedef void (*AccessBridge_PropertySelectionChangeFP) (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_PropertyTextChangeFP) (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_PropertyCaretChangeFP) (long vmID, JOBJECT64 event, JOBJECT64 source,
                                                                                                int oldPosition, int newPosition);
typedef void (*AccessBridge_PropertyVisibleDataChangeFP)  (long vmID, JOBJECT64 event, JOBJECT64 source);
typedef void (*AccessBridge_PropertyChildChangeFP) (long vmID, JOBJECT64 event, JOBJECT64 source,
                                                                                                JOBJECT64 oldChild, JOBJECT64 newChild);
typedef void (*AccessBridge_PropertyActiveDescendentChangeFP) (long vmID, JOBJECT64 event,
                                                               JOBJECT64 source,
                                                               JOBJECT64 oldActiveDescendent,
                                                               JOBJECT64 newActiveDescendent);

typedef void (*AccessBridge_PropertyTableModelChangeFP) (long vmID, JOBJECT64 event, JOBJECT64 src,
                                                                                                             wchar_t *oldValue, wchar_t *newValue);

#ifdef __cplusplus
}
#endif

#endif
