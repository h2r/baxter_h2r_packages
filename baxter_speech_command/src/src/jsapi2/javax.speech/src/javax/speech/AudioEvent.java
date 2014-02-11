/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 68 $
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

// Comp. 2.0.6

public class AudioEvent extends SpeechEvent {
    public static final int AUDIO_STARTED = 0x8000001;

    public static final int AUDIO_STOPPED = 0x8000002;

    public static final int AUDIO_CHANGED = 0x8000004;

    public static final int AUDIO_LEVEL = 0x8000008;

    public static final int DEFAULT_MASK = AUDIO_STARTED | AUDIO_CHANGED |  AUDIO_STOPPED;

    public static final int AUDIO_LEVEL_MIN = 0;

    public static final int AUDIO_LEVEL_QUIET = 250;

    public static final int AUDIO_LEVEL_LOUD = 750;

    public static final int AUDIO_LEVEL_MAX = 1000;

    private int audioLevel;

    private String mediaLocator;

    public AudioEvent(Engine source, int id) {
        super(source, id);
        if ((id != AUDIO_STARTED) && (id != AUDIO_CHANGED) && (id != AUDIO_STOPPED)) {
            throw new IllegalArgumentException(
                    "Id must be AUDIO_STARTED, AUDIO_CHANGED or AUDIO_STOPPED!");
        }
        audioLevel = AUDIO_LEVEL_MIN;
    }

    public AudioEvent(Engine source, int id, int audioLevel) {
        super(source, id);
        if (id != AUDIO_LEVEL) {
            throw new IllegalArgumentException("Id must be AUDIO_LEVEL!");
        }

        if ((audioLevel < AUDIO_LEVEL_MIN) || (audioLevel > AUDIO_LEVEL_MAX)) {
            throw new IllegalArgumentException("Audiolevel must be between "
                    + "AUDIO_LEVEL_MIN and AUDIO_LEVEL_MAX");
        }

        this.audioLevel = audioLevel;
    }

    public AudioEvent(Engine source, int id, String locator) {
        this(source, id);
        if (id != AUDIO_CHANGED) {
            throw new IllegalArgumentException("Id must be AUDIO_CHANGED!");
        }
        mediaLocator = locator;
        audioLevel = AUDIO_LEVEL_MIN;
    }

    public int getAudioLevel() {
        return audioLevel;
    }

    public String getMediaLocator() {
        return mediaLocator;
    }

    /**
     * {@inheritDoc}
     */
    protected void id2String(StringBuffer str) {
        maybeAddId(str, AUDIO_STARTED, "AUDIO_STARTED");
        maybeAddId(str, AUDIO_STOPPED, "AUDIO_STOPPED");
        maybeAddId(str, AUDIO_LEVEL, "AUDIO_LEVEL");
        super.id2String(str);
    }

    
    /**
     * {@inheritDoc}
     */
    protected Vector getParameters() {
        final Vector parameters = super.getParameters();

        final Integer level = new Integer(audioLevel);
        parameters.addElement(level);

        return parameters;
    }
}
