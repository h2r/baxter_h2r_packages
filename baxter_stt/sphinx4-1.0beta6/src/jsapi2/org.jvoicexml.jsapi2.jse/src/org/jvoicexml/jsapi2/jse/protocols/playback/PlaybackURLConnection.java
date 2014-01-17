/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2.jse/src/org/jvoicexml/jsapi2/jse/protocols/playback/PlaybackURLConnection.java $
 * Version: $LastChangedRevision: 298 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An base implementation for JSR 113.
 *
 * Copyright (C) 2009 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */

package org.jvoicexml.jsapi2.jse.protocols.playback;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.URISyntaxException;
import java.net.URL;
import java.net.URLConnection;
import java.net.UnknownServiceException;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.SourceDataLine;

import org.jvoicexml.jsapi2.jse.protocols.JavaSoundParser;

/**
 * A {@link URLConnection} for the playback protocol.
 * @author Renato Cassaca
 * @author Dirk Schnelle-Walka
 * @version 1.0
 */
public class PlaybackURLConnection extends URLConnection {
    /** Logger for this class. */
    private static final Logger LOGGER =
            Logger.getLogger(PlaybackURLConnection.class.getName());

    /** Microphone access point. */
    private SourceDataLine line;

    /** Write point given to clients. */
    private OutputStream outputStream;

    /** The audio format to use. */
    private AudioFormat audioFormat;

    /**
     * Constructs a new object.
     *
     * @param url
     *            URL
     */
    public PlaybackURLConnection(final URL url) {
        super(url);

        // Validate the kind of input supported
        if (!url.getAuthority().equals("audio")) {
            throw new UnsupportedOperationException(
                    "Can only process 'audio'. " + url.getAuthority()
                            + " is unsupported");
        }
    }

    /**
     * {@inheritDoc}
     *
     * Closes any open line.
     */
    protected void finalize() throws Throwable {
        if (outputStream != null) {
            try {
                outputStream.close();
            } catch (IOException e) {
                if (LOGGER.isLoggable(Level.FINE)) {
                    LOGGER.fine(e.getMessage());
                }
            }
            outputStream = null;
        }
        if (line != null) {
            if (line.isOpen()) {
                line.close();
            }
            line = null;
        }
    }

    /**
     * {@inheritDoc}
     */
    public void connect() throws IOException {
        if (connected) {
            return;
        }
        // Get audio format that will open playback
        AudioFormat format = getAudioFormat();

        // Representation of the line that will be opened
        DataLine.Info info = new DataLine.Info(SourceDataLine.class,
                format);

        // Checks if line is supported
        if (!AudioSystem.isLineSupported(info)) {
            throw new IOException("Cannot open the requested line: "
                    + info.toString());
        }

        // Obtain, open and start the line.
        try {
            line = (SourceDataLine) AudioSystem.getLine(info);
            line.open(format, AudioSystem.NOT_SPECIFIED);

            // Starts the line
            line.start();
        } catch (LineUnavailableException ex) {
            throw new IOException("Line is unavailable: "
                    + ex.getMessage(), ex);
        }

        // Marks this URLConnection as connected
        connected = true;
    }

    /**
     * Given URI parameters, constructs an AudioFormat.
     *
     * @return AudioFormat
     * @exception IOException
     *            error determining the audio format.
     */
    public AudioFormat getAudioFormat() throws IOException {
        if (audioFormat == null) {
            final URL url = getURL();
            try {
                audioFormat = JavaSoundParser.parse(url);
            } catch (URISyntaxException e) {
                throw new IOException(e.getMessage());
            }
        }

        return audioFormat;
    }

    /**
     * {@inheritDoc}
     * Throws an {@link UnknownServiceException}.
     *
     * @throws IOException
     *         input streams are not supported by the capture protocol.
     */
    public InputStream getInputStream()
        throws IOException {
        throw new UnknownServiceException("Cannot read from a playback device");
    }

    /**
     * {@inheritDoc}
     */
    public OutputStream getOutputStream() throws IOException {
        if (outputStream == null) {
            outputStream = new LineOutputStream(line);
        }
        return outputStream;
    }
}
