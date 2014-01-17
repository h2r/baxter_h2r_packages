/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2.jse/src/org/jvoicexml/jsapi2/jse/protocols/capture/CaptureURLConnection.java $
 * Version: $LastChangedRevision: 292 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An base implementation for JSR 113.
 *
 * Copyright (C) 2009 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */

package org.jvoicexml.jsapi2.jse.protocols.capture;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.net.URISyntaxException;
import java.net.URL;
import java.net.URLConnection;
import java.net.URLDecoder;
import java.net.UnknownServiceException;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.Line;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.Mixer;
import javax.sound.sampled.TargetDataLine;
import javax.sound.sampled.Mixer.Info;

import org.jvoicexml.jsapi2.jse.protocols.JavaSoundParser;

/**
 * A {@link URLConnection} for the capture protool.
 * @author Renato Cassaca
 * @author Dirk Schnelle-Walka
 * @version 1.0
 */
public class CaptureURLConnection extends URLConnection {
    /** Logger for this class. */
    private static final Logger LOGGER =
            Logger.getLogger(CaptureURLConnection.class.getName());

    /** Device that that this URLConnection will connect to. */
    private final String deviceName;

    /** The audio format to use. */
    private AudioFormat audioFormat;

    /** The current line. */
    private TargetDataLine line;

    /** The audio input stream. */
    private AudioInputStream inputStream;

    /**
     * Constructs a new object.
     *
     * @param url
     *            URL
     */
    public CaptureURLConnection(final URL url) {
        super(url);

        // Initialize the device that will be connecting to
        try {
            final String authority = url.getAuthority();
            deviceName = URLDecoder.decode(authority, "UTF-8");
        } catch (UnsupportedEncodingException ex) {
            throw new UnsupportedOperationException(ex);
        }
    }

    /**
     * {@inheritDoc}
     *
     * Closes any open line.
     */
    protected void finalize() throws Throwable {
        if (inputStream != null) {
            try {
                inputStream.close();
            } catch (IOException e) {
                if (LOGGER.isLoggable(Level.FINE)) {
                    LOGGER.fine(e.getMessage());
                }
            }
            inputStream = null;
        }
        if (line != null) {
            if (line.isOpen()) {
                line.close();
            }
            line = null;
        }
    }

    /**
     * Opens a communications link to the resource referenced by this URL, if
     * such a connection has not already been established.
     *
     * @throws IOException
     *             if an I/O error occurs while opening the connection.
     * @todo Implement this java.net.URLConnection method
     */
    public synchronized void connect() throws IOException {
        if (connected) {
            return;
        }

        // Get the mixer info associated with the device name
        line = getLine();
        if (line == null) {
            throw new IOException("Cannot open line with required format: "
                    + getAudioFormat());
        }

        // Obtain, open and start the line.
        try {
            line.open(audioFormat, AudioSystem.NOT_SPECIFIED);

            // Starts the line
            line.start();
        } catch (LineUnavailableException ex) {
            throw new IOException("Line is unavailable");
        }

        // Marks this URLConnection as connected
        connected = true;

    }

    /**
     * Retrieves the line to use.
     *
     * @return the line to use.
     * @exception IOException
     *            error obtaining the line.
     */
    private TargetDataLine getLine() throws IOException {

        if (deviceName.equals("audio")) {
            try {
                line = AudioSystem.getTargetDataLine(getAudioFormat());
            } catch (LineUnavailableException ex) {
                ex.printStackTrace();
                return null;
            }
        } else {
            // Get the mixer info for the device
            Info mixerInfo = getMixerInfo();
            if (mixerInfo == null) {
                return null;
            }

            Mixer mixer = AudioSystem.getMixer(mixerInfo);
            try {
                mixer.open();
            } catch (LineUnavailableException ex1) {
                ex1.printStackTrace();
            }

            Line[] liness = mixer.getTargetLines();
            for (Line lll : liness) {
                System.out.println("Infos: " + lll);
            }

            // Info[] infos = AudioSystem.getTargetLineInfo(mixerInfo);
            DataLine.Info lineInfo = new DataLine.Info(TargetDataLine.class,
                    getAudioFormat());

            try {
                line = (TargetDataLine) mixer.getLine(lineInfo);
            } catch (LineUnavailableException ex) {
                ex.printStackTrace();
                return null;
            }
        }

        return line;
    }

    /**
     * Given a mixer name, returns the mixer info that matches.
     *
     * @return Info
     */
    private Info getMixerInfo() {
        Info[] mixerInfo = AudioSystem.getMixerInfo();
        for (Info info : mixerInfo) {
            if (info.getName().contains(deviceName)) {
                if (!info.getName().contains("Playback")
                        && !info.getDescription().contains("Playback")) {
                    return info;
                }
            }
        }
        return null;
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
     */
    public InputStream getInputStream() throws IOException {
        // Get line associated with connection
        if (line == null) {
            throw new IOException("Not connected to line");
        }

        // Setup the input stream
        if (inputStream == null) {
            inputStream = new AudioInputStream(line);
        }
        return inputStream;
    }

    /**
     * {@inheritDoc}
     * Throws an {@link UnknownServiceException}.
     *
     * @throws UnknownServiceException
     *         output streams are not supported by the capture protocol.
     */
    public OutputStream getOutputStream() throws UnknownServiceException {
        throw new UnknownServiceException("Cannot write to a capture device");
    }

}
