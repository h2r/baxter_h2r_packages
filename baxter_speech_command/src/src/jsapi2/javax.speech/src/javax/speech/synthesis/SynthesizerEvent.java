/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 65 $
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

package javax.speech.synthesis;

import java.util.Vector;

import javax.speech.EngineEvent;

//Comp. 2.0.6

public class SynthesizerEvent extends EngineEvent {
    public static final int QUEUE_EMPTIED = 0x3000800;

    public static final int QUEUE_UPDATED = 0x3001000;

    public static final int SYNTHESIZER_BUFFER_UNFILLED = 0x3002000;

    public static final int SYNTHESIZER_BUFFER_READY = 0x50348032;

    public static final int DEFAULT_MASK = EngineEvent.DEFAULT_MASK | QUEUE_EMPTIED
            | SYNTHESIZER_BUFFER_UNFILLED | SYNTHESIZER_BUFFER_READY;

    private boolean topOfQueueChanged;

    public SynthesizerEvent(Synthesizer source, int id, long oldEngineState,
            long newEngineState, Throwable problem, boolean topOfQueueChanged)
        throws IllegalArgumentException {
        super(source, id, oldEngineState, newEngineState, problem);
        if ((id != ENGINE_ERROR) && (problem != null)) {
            throw new IllegalArgumentException(
                    "A problem can only be specified for ENGINE_ERROR");
        }
        if (topOfQueueChanged && (id != QUEUE_UPDATED)
                && (id != QUEUE_EMPTIED)) {
            throw new IllegalArgumentException(
                    "topOfQueueChanged can only be set for the events" +
                    " QUEUE_UPDATED and QUEUE_EMPTIED");
        }
        this.topOfQueueChanged = topOfQueueChanged;
    }

    public boolean isTopOfQueueChanged() {
        final int id = getId();
        if (id == QUEUE_UPDATED) {
            return topOfQueueChanged;
        }

        return false;
    }

    /**
     * {@inheritDoc}
     */
    protected void id2String(StringBuffer str) {
        maybeAddId(str, QUEUE_EMPTIED, "QUEUE_EMPTIED");
        maybeAddId(str, QUEUE_UPDATED, "QUEUE_UPDATED");
        maybeAddId(str, SYNTHESIZER_BUFFER_READY, 
                "SYNTHESIZER_BUFFER_READY");
        maybeAddId(str, SYNTHESIZER_BUFFER_UNFILLED, 
            "SYNTHESIZER_BUFFER_UNFILLED");
        super.id2String(str);
    }

    /**
     * {@inheritDoc}
     */
    protected Vector getParameters() {
        final Vector parameters = super.getParameters();

        final Boolean topOfQueueChangedObject = new Boolean(topOfQueueChanged);
        parameters.addElement(topOfQueueChangedObject);

        return parameters;
    }
}
