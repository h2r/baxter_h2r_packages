/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2.jse/src/org/jvoicexml/jsapi2/jse/synthesis/freetts/FreeTTSSpeakable.java $
 * Version: $LastChangedRevision: 292 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An base implementation for JSR 113.
 *
 * Copyright (C) 2009 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */

package org.jvoicexml.jsapi2.synthesis;

import javax.speech.synthesis.Speakable;

/**
 * A basic implementation of a {@link Speakable}.
 *
 * @author Renato Cassaca
 * author Dirk Schnelle-Walka
 */
public class BaseSpeakable implements Speakable {
    /** The markup. */
    private String markup;


    /**
     * Constructs a new object.
     * @param text the markup.
     */
    public BaseSpeakable(final String text) {
        markup = text;
    }

    /**
     * {@inheritDoc}
     */
    public final String getMarkupText() {
        return markup;
    }
}
