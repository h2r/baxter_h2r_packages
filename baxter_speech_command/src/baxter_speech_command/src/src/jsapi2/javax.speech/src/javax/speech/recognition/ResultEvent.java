/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 63 $
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

import javax.speech.SpeechEvent;

// Comp 2.0.6

public class ResultEvent extends SpeechEvent {

    public static final int RESULT_CREATED = 0x6000001;

    public static final int RESULT_UPDATED = 0x6000002;

    public static final int GRAMMAR_FINALIZED = 0x6000004;

    public static final int RESULT_ACCEPTED = 0x6000008;

    public static final int RESULT_REJECTED = 0x6000010;

    public static final int AUDIO_RELEASED = 0x6000020;

    public static final int TRAINING_INFO_RELEASED = 0x6000040;

    public static final int DEFAULT_MASK = RESULT_CREATED | RESULT_UPDATED;

    private boolean tokensFinalized;

    private boolean unfinalizedTokensChanged;

    public ResultEvent(Result source, int id) throws IllegalArgumentException {
        super(source, id);
    }

    public ResultEvent(Result source, int id, boolean tokensFinalized,
            boolean unfinalizedTokensChanged) throws IllegalArgumentException {
        super(source, id);

        this.tokensFinalized = tokensFinalized;
        this.unfinalizedTokensChanged = unfinalizedTokensChanged;
    }

    public boolean isFinalizedChanged() {
        final int id = getId();
        if ((id == RESULT_CREATED) || (id == RESULT_UPDATED)
                || (id == RESULT_ACCEPTED) || (id == RESULT_REJECTED)) {
            return tokensFinalized;
        }

        return false;
    }

    public boolean isUnfinalizedChanged() {
        final int id = getId();
        if ((id == RESULT_CREATED) || (id == RESULT_UPDATED)
                || (id == RESULT_ACCEPTED) || (id == RESULT_REJECTED)) {
            return tokensFinalized;
        }

        return false;
    }

    /**
     * {@inheritDoc}
     */
    protected Vector getParameters() {
        final Vector parameters = super.getParameters();

        final Boolean tokensFinalizedObject = new Boolean(tokensFinalized);
        parameters.addElement(tokensFinalizedObject);
        final Boolean unfinalizedTokensChangedObject = new Boolean(
                unfinalizedTokensChanged);
        parameters.addElement(unfinalizedTokensChangedObject);

        return parameters;
    }
}
