/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 66 $
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

import javax.speech.EngineMode;
import javax.speech.SpeechLocale;

//Comp 2.0.6

public class SynthesizerMode extends EngineMode {
    /**
     * Returns a hash code value for the array
     * @param array the array to create a hash code value for
     * @return a hash code value for the array
     */
    private static int hashCode(Object[] array) {
        int prime = 31;
        if (array == null)
            return 0;
        int result = 1;
        for (int index = 0; index < array.length; index++) {
            result = prime * result
                    + (array[index] == null ? 0 : array[index].hashCode());
        }
        return result;
    }

    public static final SynthesizerMode DEFAULT = new SynthesizerMode();

    private Voice[] voices;

    public SynthesizerMode() {
        super();
    }

    public SynthesizerMode(SpeechLocale locale) {
        super();

        voices = new Voice[1];
        voices[0] = new Voice(locale, null, Voice.GENDER_DONT_CARE,
                Voice.AGE_DONT_CARE, Voice.VARIANT_DONT_CARE);
    }

    public SynthesizerMode(String engineName, String modeName, Boolean running,
            Boolean supportsLetterToSound, Boolean supportsMarkup,
            Voice[] voices) {
        super(engineName, modeName, running, supportsLetterToSound,
                supportsMarkup);

        this.voices = voices;
    }


    /* (non-Javadoc)
     * @see java.lang.Object#hashCode()
     */
    public int hashCode() {
        final int prime = 31;
        int result = super.hashCode();
        result = prime * result + SynthesizerMode.hashCode(voices);
        return result;
    }

    /* (non-Javadoc)
     * @see java.lang.Object#equals(java.lang.Object)
     */
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!super.equals(obj)) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        SynthesizerMode other = (SynthesizerMode) obj;
        if (voices == null) {
            if (other.voices != null) {
                return false;
            }
        } else if (voices.length != other.voices.length) {
            return false;
        } else {
            for (int i=0; i<voices.length; i++) {
                if (!voices[i].equals(other.voices[i])) {
                    return false;
                }
            }
        }
        return true;
    }

    public Boolean getMarkupSupport() {
        return super.getSupportsMarkup();
    }

    public Voice[] getVoices() {
        return voices;
    }

    public boolean match(EngineMode require) {
        if (!super.match(require)) {
            return false;
        }

        if (require instanceof SynthesizerMode) {
            final SynthesizerMode mode = (SynthesizerMode) require;
            Voice[] otherVoices = mode.getVoices();
            if (otherVoices != null) {
                if (voices == null) {
                    return false;
                }

                for (int i = 0; i < otherVoices.length; i++) {
                    Voice otherVoice = otherVoices[i];

                    boolean voiceMatch = false;
                    for (int k = 0; (k < voices.length) && !voiceMatch; k++) {
                        final Voice voice = voices[k];
                        if (otherVoice.match(voice)) {
                            voiceMatch = true;
                        }
                    }

                    if (!voiceMatch) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    /**
     * Creates a collection of all parameters.
     * 
     * @return collection of all parameters.
     */
    protected Vector getParameters() {
        final Vector parameters = super.getParameters();

        if (voices == null) {
            parameters.addElement(null);
        } else {
            final Vector col = new Vector();
            for (int i = 0; i < voices.length; i++) {
                col.addElement(voices[i]);
            }
            parameters.addElement(col);
        }

        return parameters;
    }
}
