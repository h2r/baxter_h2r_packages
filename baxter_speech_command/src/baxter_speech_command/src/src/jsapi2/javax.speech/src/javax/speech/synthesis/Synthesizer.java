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

import javax.speech.AudioSegment;
import javax.speech.Engine;
import javax.speech.EngineStateException;

//Comp 2.0.6

public interface Synthesizer extends Engine {
    long QUEUE_EMPTY = 0x1000;

    long QUEUE_NOT_EMPTY = 0x2000;

    void addSpeakableListener(SpeakableListener listener);

    void removeSpeakableListener(SpeakableListener listener);

    void addSynthesizerListener(SynthesizerListener listener);

    void removeSynthesizerListener(SynthesizerListener listener);

    boolean cancel() throws EngineStateException;

    boolean cancel(int id) throws EngineStateException;

    boolean cancelAll() throws EngineStateException;

    String getPhonemes(String text) throws EngineStateException;

    SynthesizerProperties getSynthesizerProperties();

    void pause() throws EngineStateException;

    boolean resume() throws EngineStateException;

    void setSpeakableMask(int mask);

    int getSpeakableMask();

    int speak(AudioSegment audio, SpeakableListener listener)
            throws SpeakableException, EngineStateException,
                IllegalArgumentException;

    int speak(Speakable speakable, SpeakableListener listener)
            throws SpeakableException, EngineStateException;

    int speak(String text, SpeakableListener listener)
            throws EngineStateException;

    int speakMarkup(String synthesisMarkup, SpeakableListener listener)
            throws SpeakableException, EngineStateException;
}
