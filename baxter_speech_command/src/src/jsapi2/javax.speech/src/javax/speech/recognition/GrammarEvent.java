/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 58 $
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

public class GrammarEvent extends SpeechEvent {
    public static int GRAMMAR_CHANGES_COMMITTED = 0x4000001;

    public static int GRAMMAR_ACTIVATED = 0x4000002;

    public static int GRAMMAR_DEACTIVATED = 0x4000004;

    public static int GRAMMAR_CHANGES_REJECTED = 0x4000008;

    public static int DEFAULT_MASK = GRAMMAR_ACTIVATED
            | GRAMMAR_CHANGES_COMMITTED | GRAMMAR_CHANGES_REJECTED
            | GRAMMAR_DEACTIVATED;

    private boolean activableChanged;

    private boolean definitionChanged;

    private GrammarException grammarException;

    public GrammarEvent(Object source, int id) throws IllegalArgumentException {
        super(source, id);
    }

    public GrammarEvent(Grammar source, int id, boolean activableChanged,
            boolean definitionChanged, GrammarException grammarException) 
        throws IllegalArgumentException {
        super(source, id);
        if ((id != GRAMMAR_CHANGES_REJECTED) && (grammarException != null)) {
            throw new IllegalArgumentException(
                    "A grammar exception can only supplied for "
                    + "GRAMMAR_CHANGES_REJECTED!");
        }
        this.activableChanged = activableChanged;
        this.definitionChanged = definitionChanged;
        this.grammarException = grammarException;
    }

    public GrammarException getGrammarException() {
        final int id = getId();
        if (id == GRAMMAR_CHANGES_REJECTED) {
            return grammarException;
        }

        return null;
    }

    public boolean isDefinitionChanged() {
        return definitionChanged;
    }

    public boolean isActivableChanged() {
        return activableChanged;
    }

    /**
     * {@inheritDoc}
     */
    protected Vector getParameters() {
        final Vector parameters = super.getParameters();

        final Boolean definitionChangedObject = new Boolean(definitionChanged);
        parameters.addElement(definitionChangedObject);
        final Boolean enabledChangedObject = new Boolean(activableChanged);
        parameters.addElement(enabledChangedObject);
        parameters.addElement(grammarException);

        return parameters;
    }
}
