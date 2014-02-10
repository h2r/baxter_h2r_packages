/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 55 $
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

//Comp. 2.0.6

public class Word {
    public static final long UNKNOWN = 0x00000;

    public static final long DONT_CARE = 0x00001;

    public static final long OTHER = 0x00002;

    public static final long NOUN = 0x00004;

    public static final long PROPER_NOUN = 0x00008;

    public static final long PRONOUN = 0x00010;

    public static final long VERB = 0x00020;

    public static final long ADVERB = 0x00040;

    public static final long ADJECTIVE = 0x00080;

    public static final long PROPER_ADJECTIVE = 0x00100;

    public static final long AUXILIARY = 0x00200;

    public static final long DETERMINER = 0x00400;

    public static final long CARDINAL = 0x00800;

    public static final long CONJUNCTION = 0x01000;

    public static final long PREPOSITION = 0x02000;

    public static final long CONTRACTION = 0x04000;

    public static final long ABBREVIATION = 0x08000;

    public static final long ACOUSTIC = 0x10000;

    private String text;

    private String[] pronunciations;

    private String spokenForm;

    private AudioSegment audioSegment;

    private long categories;

    private SpeechLocale locale;

    public Word(String text, String[] pronunciations, String spokenForm,
            AudioSegment audioSegment, long categories)
        throws IllegalArgumentException {
        this(text, pronunciations, spokenForm, audioSegment, categories, null);
    }

    public Word(String text, String[] pronunciations, String spokenForm,
            AudioSegment audioSegment, long categories, SpeechLocale locale)
        throws IllegalArgumentException {
        if (text == null) {
            throw new IllegalArgumentException(
                    "Written form text must be specified");
        }
        this.text = text;
        this.pronunciations = pronunciations;
        this.spokenForm = spokenForm;
        this.audioSegment = audioSegment;
        this.categories = categories;
        this.locale = locale;
    }

    public AudioSegment getAudioSegment() {
        return audioSegment;
    }

    public long getCategories() {
        return categories;
    }

    public String[] getPronunciations() {
        return pronunciations;
    }

    public String getSpokenForm() {
        return spokenForm;
    }

    public String getText() {
        return text;
    }

    public SpeechLocale getSpeechLocale() {
        return locale;
    }

    public String toString() {
        final StringBuffer str = new StringBuffer();

        str.append(getClass());
        str.append("[");

        str.append(text);

        if (pronunciations == null) {
            str.append(pronunciations);
        } else {
            str.append("[");
            int max = pronunciations.length;
            for (int i = 0; i < max; i++) {
                str.append(pronunciations[i]);
                if (i != max - 1) {
                    str.append(",");
                }
            }
            str.append("]");
        }

        str.append(spokenForm);
        str.append(audioSegment);
        str.append(categories);

        str.append("]");

        return str.toString();
    }
}
