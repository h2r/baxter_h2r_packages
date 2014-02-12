/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 62 $
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

import javax.speech.EngineProperties;

//Comp 2.0.6

public interface RecognizerProperties extends EngineProperties {
    int UNKNOWN_CONFIDENCE = -1;

    int MIN_CONFIDENCE = 0;

    int NORM_CONFIDENCE = 5;

    int MAX_CONFIDENCE = 10;

    int MIN_SENSITIVITY = 0;

    int NORM_SENSITIVITY = 5;

    int MAX_SENSITIVITY = 10;

    int MIN_ACCURACY = 0;

    int NORM_ACCURACY = 5;

    int MAX_ACCURACY = 10;

    int ADAPT_PAUSED = 1;

    int ADAPT_RESUMED = 2;

    int ENDPOINT_AUTOMATIC = 0x01;

    int ENDPOINT_MANUAL = 0x02;

    int ENDPOINT_SPEECH_DETECTION = 0x04;

    int ENDPOINT_PUSH_TO_TALK = 0x08;

    int ENDPOINT_PUSH_TO_START = 0x10;

    void setAdaptation(int adapt) throws IllegalArgumentException;

    int getAdaptation();

    void setCompleteTimeout(int timeout) throws IllegalArgumentException;

    int getCompleteTimeout();

    void setConfidenceThreshold(int confidenceThreshold)
        throws IllegalArgumentException;

    int getConfidenceThreshold();

    void setEndpointStyle(int endpointStyle) throws IllegalArgumentException;

    int getEndpointStyle();

    void setIncompleteTimeout(int timeout) throws IllegalArgumentException;

    int getIncompleteTimeout();

    void setNumResultAlternatives(int num) throws IllegalArgumentException;

    int getNumResultAlternatives();

    void setPriority(int priority) throws IllegalArgumentException;

    int getPriority();

    int getSensitivity();

    void setSensitivity(int sensitivity) throws IllegalArgumentException;

    void setSpeedVsAccuracy(int speedVsAccuracy)
        throws IllegalArgumentException;

    int getSpeedVsAccuracy();

    void setResultAudioProvided(boolean audioProvided);

    boolean isResultAudioProvided();

    void setTrainingProvided(boolean trainingProvided);

    boolean isTrainingProvided();
}
