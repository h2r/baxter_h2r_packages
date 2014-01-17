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

package javax.speech.recognition;

import java.util.Vector;

import javax.speech.EngineEvent;

//Comp. 2.0.6

public class RecognizerEvent extends EngineEvent {
    public static final int RECOGNIZER_PROCESSING = 0x2000800;

    public static final int RECOGNIZER_LISTENING = 0x2001000;

    public static final int CHANGES_COMMITTED = 0x2002000;

    public static final int CHANGES_REJECTED = 0x2040000;

    public static final int SPEECH_STARTED = 0x2004000;

    public static final int SPEECH_STOPPED = 0x2008000;

    public static final int RECOGNIZER_NOT_BUFFERING = 0x2010000;

    public static final int RECOGNIZER_BUFFERING = 0x2020000;

    public static final int RECOGNIZER_STOPPED = 0x2040000;

    public static final int UNKNOWN_AUDIO_POSITION = -1;

    public static final int DEFAULT_MASK = EngineEvent.DEFAULT_MASK
            | CHANGES_COMMITTED | CHANGES_REJECTED | RECOGNIZER_LISTENING
            | RECOGNIZER_PROCESSING | SPEECH_STARTED | SPEECH_STOPPED;

    private GrammarException grammarException;

    private long audioPosition;

    public RecognizerEvent(Recognizer source, int id, long oldEngineState,
            long newEngineState, Throwable problem,
            GrammarException grammarException, long audioPosition) 
        throws IllegalArgumentException {
        super(source, id, oldEngineState, newEngineState, problem);
        if ((id != ENGINE_ERROR) && (problem != null)) {
            throw new IllegalArgumentException(
                    "A problem can only be specified for ENGINE_ERROR");
        }
        if ((id == SPEECH_STARTED) || (id == SPEECH_STOPPED)
                || (id == RECOGNIZER_BUFFERING)
                || (id == RECOGNIZER_NOT_BUFFERING)) {
            if (audioPosition < 0) {
                throw new IllegalArgumentException(
                        "Audio position must be a non-negative integer!");
            }
        } else {
            if (audioPosition != UNKNOWN_AUDIO_POSITION) {
                StringBuffer str = new StringBuffer();
                id2String(str);
                throw new IllegalArgumentException(
                        "Audio position must be UNKNOWN_AUDIO_POSITION for"
                        + " the given event id (" + str.toString() + ")!");
            }
        }
        this.grammarException = grammarException;
        this.audioPosition = audioPosition;
    }

    public long getAudioPosition() {
        return audioPosition;
    }

    public GrammarException getGrammarException() {
        final int id = getId();
        if (id == CHANGES_REJECTED) {
            return grammarException;
        }

        return null;
    }

    /**
     * {@inheritDoc}
     */
    protected Vector getParameters() {
        final Vector parameters = super.getParameters();

        final Long audioPositionObject = new Long(audioPosition);
        parameters.addElement(audioPositionObject);
        parameters.addElement(grammarException);

        return parameters;
    }
}
