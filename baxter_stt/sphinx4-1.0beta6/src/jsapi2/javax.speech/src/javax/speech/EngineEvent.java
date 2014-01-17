/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 54 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An independent reference implementation of JSR 113.
 *
 * Copyright (C) 2007 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

package javax.speech;

import java.util.Vector;

//Comp. 2.0.6

public class EngineEvent extends SpeechEvent {
    // Events.
    public static final int ENGINE_ALLOCATED = 0x1000001;

    public static final int ENGINE_DEALLOCATED = 0x1000002;

    public static final int ENGINE_ALLOCATING_RESOURCES = 0x1000004;

    public static final int ENGINE_DEALLOCATING_RESOURCES = 0x1000008;

    public static final int ENGINE_PAUSED = 0x1000010;

    public static final int ENGINE_RESUMED = 0x1000020;

    public static final int ENGINE_DEFOCUSED = 0x1000040;

    public static final int ENGINE_FOCUSED = 0x1000080;

    public static final int ENGINE_ERROR = 0x1000100;

    public static final int DEFAULT_MASK = ENGINE_ALLOCATED
            | ENGINE_DEALLOCATED | ENGINE_PAUSED | ENGINE_RESUMED
            | ENGINE_FOCUSED | ENGINE_DEFOCUSED | ENGINE_ERROR;

    private long oldEngineState;

    private long newEngineState;

    private Throwable problem;

    public EngineEvent(Engine source, int id, long oldEngineState,
            long newEngineState, Throwable problem)
        throws IllegalArgumentException {
        super(source, id);

        if ((problem != null) && (id != ENGINE_ERROR)) {
            throw new IllegalArgumentException(
                    "A problem can only be provided for ENGINE_ERROR");
        }
        if ((problem == null) && (id == ENGINE_ERROR)) {
            throw new IllegalArgumentException(
                    "A problem must be provided for ENGINE_ERROR");
        }
        this.oldEngineState = oldEngineState;
        this.newEngineState = newEngineState;
        this.problem = problem;
    }

    public long getNewEngineState() {
        return newEngineState;
    }

    public long getOldEngineState() {
        return oldEngineState;
    }

    public Throwable getEngineError() {
        final int id = getId();
        if (id == ENGINE_ERROR) {
            return problem;
        }

        return null;
    }

    /**
     * {@inheritDoc}
     */
    protected void id2String(StringBuffer str) {
        maybeAddId(str, ENGINE_ALLOCATED, "ENGINE_ALLOCATED");
        maybeAddId(str, ENGINE_DEALLOCATED, "ENGINE_DEALLOCATED");
        maybeAddId(str, ENGINE_ALLOCATING_RESOURCES, 
                "ENGINE_ALLOCATING_RESOURCES");
        maybeAddId(str, ENGINE_DEALLOCATING_RESOURCES, 
            "ENGINE_DEALLOCATING_RESOURCES");
        maybeAddId(str, ENGINE_PAUSED, "ENGINE_PAUSED");
        maybeAddId(str, ENGINE_RESUMED, "ENGINE_RESUMED");
        maybeAddId(str, ENGINE_DEFOCUSED, "ENGINE_DEFOCUSED");
        maybeAddId(str, ENGINE_FOCUSED, "ENGINE_FOCUSED");
        maybeAddId(str, ENGINE_ERROR, "ENGINE_ERROR");
        super.id2String(str);
    }

    /**
     * {@inheritDoc}
     */
    protected Vector getParameters() {
        final Vector parameters = super.getParameters();

        final Long oldEngineStateObject = new Long(oldEngineState);
        parameters.addElement(oldEngineStateObject);
        final Long newEngineStateObject = new Long(newEngineState);
        parameters.addElement(newEngineStateObject);
        parameters.addElement(problem);

        return parameters;
    }
}
