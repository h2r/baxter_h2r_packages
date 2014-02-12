/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 53 $
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

public interface Engine {
    long ALLOCATED = 0x001;

    long ALLOCATING_RESOURCES = 0x002;

    long DEALLOCATED = 0x004;

    long DEALLOCATING_RESOURCES = 0x008;

    long PAUSED = 0x010;

    long RESUMED = 0x020;

    long DEFOCUSED = 0x40;

    long FOCUSED = 0x080;

    long ERROR_OCCURRED = 0x100;

    int ASYNCHRONOUS_MODE = 0x1;

    int IMMEDIATE_MODE = 0x2;

    void allocate() throws AudioException, EngineException,
            EngineStateException, SecurityException;

    void allocate(int mode) throws IllegalArgumentException, AudioException,
        EngineException, EngineStateException, SecurityException;

    void deallocate() throws AudioException, EngineException,
            EngineStateException;

    void deallocate(int mode) throws IllegalArgumentException, AudioException,
        EngineException, EngineStateException;

    void pause() throws EngineStateException;

    boolean resume() throws EngineStateException;

    boolean testEngineState(long state) throws IllegalArgumentException;

    long waitEngineState(long state)
        throws InterruptedException, IllegalArgumentException,
            IllegalStateException;

    long waitEngineState(long state, long timeout)
        throws InterruptedException, IllegalArgumentException,
            IllegalStateException;

    AudioManager getAudioManager();

    EngineMode getEngineMode();

    long getEngineState();

    VocabularyManager getVocabularyManager();

    void setEngineMask(int mask);

    int getEngineMask();

    SpeechEventExecutor getSpeechEventExecutor();

    void setSpeechEventExecutor(SpeechEventExecutor speechEventExecutor);
}
