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

import javax.speech.SpeechEvent;

// Comp 2.0.6

public class SpeakableEvent extends SpeechEvent {
    // Events.
    public static final int TOP_OF_QUEUE = 0x7000001;

    public static final int SPEAKABLE_STARTED = 0x7000002;

    public static final int SPEAKABLE_ENDED = 0x7000004;

    public static final int SPEAKABLE_PAUSED = 0x7000008;

    public static final int SPEAKABLE_RESUMED = 0x7000010;

    public static final int SPEAKABLE_CANCELLED = 0x7000020;

    public static final int WORD_STARTED = 0x7000040;

    public static final int PHONEME_STARTED = 0x7000080;

    public static final int MARKER_REACHED = 0x7000100;

    public static final int VOICE_CHANGED = 0x7000200;
    
    public static final int SPEAKABLE_FAILED = 0x7000400;

    public static final int PROSODY_UPDATED = 0x7000800;

    public static final int ELEMENT_REACHED = 0x7001000;

    public static final int DEFAULT_MASK = MARKER_REACHED | SPEAKABLE_FAILED
            | SPEAKABLE_CANCELLED | SPEAKABLE_STARTED | SPEAKABLE_ENDED
            | SPEAKABLE_PAUSED | SPEAKABLE_RESUMED | VOICE_CHANGED;

    // Types.
    public static final int ELEMENT_OPEN = 1;

    public static final int ELEMENT_CLOSE = 2;

    public static final int SPEAKABLE_FAILURE_RECOVERABLE = 0x10;

    public static final int  SPEAKABLE_FAILURE_UNRECOVERABLE = 0x20;

    public static final int PROSODY_RATE = 0x01;

    public static final int PROSODY_PITCH = 0x02;
    
    public static final int PROSODY_VOLUME = 0x04;

    public static final int PROSODY_PITCH_RANGE = 0x08;

    public static final int PROSODY_CONTOUR = 0x10;

    public static final int UNKNOWN_AUDIO_POSITION = -1;

    public static final int UNKNOWN_INDEX = -1;

    public static final int UNKNOWN_TYPE = -1;

    public static final int UNKNOWN_VALUE = -1;

    private int requestId;

    private String textInfo;

    private int audioPosition;

    private int textBegin;

    private int textEnd;

    private int type;

    private int requested;

    private int realized;

    private SpeakableException exception;

    private String[] attributes;

    private PhoneInfo[] phones;

    private int index;

    private Voice newVoice;

    private Voice oldVoice;

    public SpeakableEvent(Object source, int id, int requestId)
        throws IllegalArgumentException {
        super(source, id);
        if ((id != TOP_OF_QUEUE) && (id != SPEAKABLE_STARTED)
                && (id != SPEAKABLE_ENDED) && (id != SPEAKABLE_PAUSED)
                && (id != SPEAKABLE_RESUMED) && (id != SPEAKABLE_CANCELLED)
                && (id != ELEMENT_REACHED) && (id != VOICE_CHANGED)
                && (id != PROSODY_UPDATED) && (id != MARKER_REACHED)
                && (id != WORD_STARTED) && (id != PHONEME_STARTED)
                && (id != SPEAKABLE_FAILED)) {
            StringBuffer str = new StringBuffer();
            id2String(str);
            throw new IllegalArgumentException("Invalid event identifier "
                    + str.toString() + "!");
        }
        this.requestId = requestId;
    }

    public SpeakableEvent(Object source, int id, int requestId,
            String textInfo, int audioPosition)
        throws IllegalArgumentException {
        this(source, id, requestId);
        if (id != MARKER_REACHED) {
            StringBuffer str = new StringBuffer();
            id2String(str);
            throw new IllegalArgumentException("Invalid event identifier "
                    + str.toString() + "!");
        }

        this.textInfo = textInfo;
        this.audioPosition = audioPosition;
    }

    public SpeakableEvent(Object source, int id, int requestId,
            String textInfo, int textBegin, int textEnd) 
        throws IllegalArgumentException {
        this(source, id, requestId);

        if (id != WORD_STARTED) {
            StringBuffer str = new StringBuffer();
            id2String(str);
            throw new IllegalArgumentException("Invalid event identifier "
                    + str.toString() + "!");
        }

        this.textInfo = textInfo;
        this.textBegin = textBegin;
        this.textEnd = textEnd;
    }

    public SpeakableEvent(Object source, int id, int requestId,
            String textInfo, int type, int requested, int realized)
        throws IllegalArgumentException {
        this(source, id, requestId);

        if (id != PROSODY_UPDATED) {
            StringBuffer str = new StringBuffer();
            id2String(str);
            throw new IllegalArgumentException("Invalid event identifier "
                    + str.toString() + "!");
        }

        this.textInfo = textInfo;
        this.type = type;
        this.requested = requested;
        this.realized = realized;
    }

    public SpeakableEvent(Object source, int id, int requestId,
            String textInfo, int type, SpeakableException exception) 
        throws IllegalArgumentException {
        this(source, id, requestId);

        if (id != SPEAKABLE_FAILED) {
            StringBuffer str = new StringBuffer();
            id2String(str);
            throw new IllegalArgumentException("Invalid event identifier "
                    + str.toString() + "!");
        }

        this.textInfo = textInfo;
        this.type = type;
        this.exception = exception;
    }

    public SpeakableEvent(Object source, int id, int requestId,
            String textInfo, int type, String[] attributes)
        throws IllegalArgumentException {
        this(source, id, requestId);

        if (id != ELEMENT_REACHED) {
            StringBuffer str = new StringBuffer();
            id2String(str);
            throw new IllegalArgumentException("Invalid event identifier "
                    + str.toString() + "!");
        }

        this.textInfo = textInfo;
        this.type = type;
        this.attributes = attributes;
    }

    public SpeakableEvent(Object source, int id, int requestId,
            String textInfo, PhoneInfo[] phones, int index) 
        throws IllegalArgumentException {
        this(source, id, requestId);

        if (id != PHONEME_STARTED) {
            StringBuffer str = new StringBuffer();
            id2String(str);
            throw new IllegalArgumentException("Invalid event identifier "
                    + str.toString() + "!");
        }

        this.textInfo = textInfo;
        this.phones = phones;
        this.index = index;
    }

    public SpeakableEvent(Object source, int id, int requestId,
            String textInfo, Voice oldVoice, Voice newVoice) 
        throws IllegalArgumentException {
        this(source, id, requestId);

        if (id != VOICE_CHANGED) {
            StringBuffer str = new StringBuffer();
            id2String(str);
            throw new IllegalArgumentException("Invalid event identifier "
                    + str.toString() + "!");
        }

        this.textInfo = textInfo;
        this.newVoice = newVoice;
        this.oldVoice = oldVoice;
    }

    public String[] getAttributes() {
        final int id = getId();
        if (id == ELEMENT_REACHED) {
            return attributes;
        }

        return new String[0];
    }

    public int getAudioPosition() {
        final int id = getId();
        if (id == MARKER_REACHED) {
            return audioPosition;
        }

        return UNKNOWN_AUDIO_POSITION;
    }

    public SpeakableException getSpeakableException() {
        return exception;
    }

    public int getIndex() {
        final int id = getId();
        if (id == PHONEME_STARTED) {
            return index;
        }

        return UNKNOWN_INDEX;
    }

    public Voice getNewVoice() {
        final int id = getId();
        if (id == VOICE_CHANGED) {
            return newVoice;
        }

        return null;
    }

    public Voice getOldVoice() {
        final int id = getId();
        if (id == VOICE_CHANGED) {
            return oldVoice;
        }

        return null;
    }

    public PhoneInfo[] getPhones() {
        final int id = getId();
        if (id == PHONEME_STARTED) {
            return phones;
        }

        return new PhoneInfo[0];
    }

    public int getRealizedValue() {
        final int id = getId();
        if (id == PROSODY_UPDATED) {
            return realized;
        }

        return UNKNOWN_VALUE;
    }

    public int getRequestedValue() {
        final int id = getId();
        if (id == PROSODY_UPDATED) {
            return requested;
        }

        return UNKNOWN_VALUE;
    }

    public int getRequestId() {
        return requestId;
    }

    public String getTextInfo() {
        return textInfo;
    }

    public int getType() {
        // TODO Check if there is an error in the specification.
        // The MARKER_REACHED does not provide a type.
        final int id = getId();
        if ((id == ELEMENT_REACHED) || (id == SPEAKABLE_FAILED)
                || (id == PROSODY_UPDATED)) {
            return type;
        }

        return UNKNOWN_TYPE;
    }

    public int getTextEnd() {
        final int id = getId();
        if (id == WORD_STARTED) {
            return textEnd;
        }

        return UNKNOWN_INDEX;
    }

    public int getTextBegin() {
        final int id = getId();
        if (id == WORD_STARTED) {
            return textBegin;
        }

        return UNKNOWN_INDEX;
    }

    /**
     * {@inheritDoc}
     */
    protected void id2String(StringBuffer str) {
        maybeAddId(str, TOP_OF_QUEUE, "TOP_OF_QUEUE");
        maybeAddId(str, SPEAKABLE_STARTED, "SPEAKABLE_STARTED");
        maybeAddId(str, ELEMENT_REACHED, "ELEMENT_REACHED");
        maybeAddId(str, VOICE_CHANGED, "VOICE_CHANGED");
        maybeAddId(str, PROSODY_UPDATED, "PROSODY_UPDATED");
        maybeAddId(str, MARKER_REACHED, "MARKER_REACHED");
        maybeAddId(str, WORD_STARTED, "WORD_STARTED");
        maybeAddId(str, PHONEME_STARTED, "PHONEME_STARTED");
        maybeAddId(str, SPEAKABLE_PAUSED, "SPEAKABLE_PAUSED");
        maybeAddId(str, SPEAKABLE_RESUMED, "SPEAKABLE_RESUMED");
        maybeAddId(str, SPEAKABLE_CANCELLED, "SPEAKABLE_CANCELLED");
        maybeAddId(str, SPEAKABLE_ENDED, "SPEAKABLE_ENDED");
        maybeAddId(str, SPEAKABLE_FAILED, "SPEAKABLE_FAILED");
        super.id2String(str);
    }

    /**
     * {@inheritDoc}
     */
    protected Vector getParameters() {
        final Vector parameters = super.getParameters();

        final int id = getId();

        final Integer typeObject = new Integer(type);
        parameters.addElement(typeObject);
        final Integer requestIdObject = new Integer(requestId);
        parameters.addElement(requestIdObject);
        if (id == PROSODY_UPDATED) {
            final Integer requestedObject = new Integer(requested);
            parameters.addElement(requestedObject);
        }
        parameters.addElement(textInfo);
        if (id == MARKER_REACHED) {
            final Integer audioPositionObject = new Integer(audioPosition);
            parameters.addElement(audioPositionObject);
        }
        if (id == WORD_STARTED) {
            final Integer wordStartObject = new Integer(textBegin);
            parameters.addElement(wordStartObject);
            final Integer wordEndObject = new Integer(textEnd);
            parameters.addElement(wordEndObject);
            parameters.addElement(newVoice);
            parameters.addElement(oldVoice);
        }
        parameters.addElement(exception);
        if (id == ELEMENT_REACHED) {
            parameters.addElement(attributes);
        }
        if (id == PHONEME_STARTED) {
            parameters.addElement(phones);
            final Integer indexObject = new Integer(index);
            parameters.addElement(indexObject);
        }

        return parameters;
    }
}
