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

import javax.speech.EngineProperties;

//Comp 2.0.6

public interface SynthesizerProperties extends EngineProperties {
    int MAX_VOLUME = 100;

    int MEDIUM_VOLUME = 50;

    int MIN_VOLUME = 0;

    int DEFAULT_VOLUME = 100;

    int WORD_LEVEL = 1;

    int OBJECT_LEVEL = 2;

    int QUEUE_LEVEL = 3;

    int DEFAULT_RATE = 0;
    
    int X_SLOW_RATE = -40;
    
    int SLOW_RATE = -70;
    
    int MEDIUM_RATE = -100;
    
    int FAST_RATE = -130;
    
    int X_FAST_RATE = -160;
    
    void setInterruptibility(int level) throws IllegalArgumentException;

    int getInterruptibility();

    void setPitch(int hertz) throws IllegalArgumentException;

    int getPitch();

    void setPitchRange(int hertz) throws IllegalArgumentException;

    int getPitchRange();

    void setSpeakingRate(int wpm) throws IllegalArgumentException;

    int getSpeakingRate();

    void setVoice(Voice voice) throws IllegalArgumentException;

    Voice getVoice();

    void setVolume(int volume) throws IllegalArgumentException;

    int getVolume();

    int getPriority();

    void setPriority(int priority) throws IllegalArgumentException;
}
