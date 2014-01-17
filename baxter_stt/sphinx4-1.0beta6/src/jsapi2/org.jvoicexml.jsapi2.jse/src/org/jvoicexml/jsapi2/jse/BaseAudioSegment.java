/*
 * File:    $HeadURL: $
 * Version: $LastChangedRevision: $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: lyncher $
 *
 * JSAPI - An base implementation for JSR 113.
 *
 * Copyright (C) 2007-2009 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */

package org.jvoicexml.jsapi2.jse;

import java.io.IOException;
import java.io.InputStream;
import java.net.URL;

import javax.speech.AudioSegment;

/**
 * Basic implementation of an {@link AudioSegment}.
 * <p>
 * The association of audio data to a representation is based on an
 * {@link InputStream} or retrieved from the given media locator.
 * </p>
 * @author Renato Cassaca
 * @version $Revision: 1370 $

 */
public class BaseAudioSegment extends AudioSegment {
    /** The audio data. */
    private final InputStream is;

    /**
     * Constructs a new audio segment without an input stream and without
     * a media locator. This means that the data can not be retrieved from
     * the locator.
     * The locator is assigned a value of <code>http://localhost/dummy</code>
     * since it must not be null.
     * @param markupText the alternate markup text
     * @param in the input stream for the audio data.
     */
    public BaseAudioSegment(final String markupText, final InputStream in) {
        this("http://localhost/dummy", markupText, in);
    }

    /**
     * Constructs a new object without an input stream. This means that
     * there is no associated audio data.
     * @param locator a non-null media locator description.
     * @param markupText the alternate markup text
     */
    public BaseAudioSegment(final String locator, final String markupText) {
        super(locator, markupText);
        is = null;
    }

    /**
     * Constructs a new object with the given input stream.
     * @param locator a non-null media locator description.
     * @param markupText the alternate markup text
     * @param input the input stream for the audio data.
     */
    public BaseAudioSegment(final String locator, final String markupText,
            final InputStream input) {
        super(locator, markupText);
        is = input;
    }

    /**
     * Opens the input stream. If an input stream is given when creating this
     * object, this one is returned, otherwise this implementation tries
     * to open a stream to the given media locator.
     * @exception IOException
     *            error opening the input stream.
     * @return the input stream with the audio data.
     */
    public InputStream openInputStream() throws IOException {
        if (!isGettable()) {
            throw new SecurityException(
                    "The platform does not allow to access the input stream!");
        }
        if (is == null) {
            final String locator = getMediaLocator();
            final URL url = new URL(locator);
            return url.openStream();
        } else {
            return is;
        }
    }

}
