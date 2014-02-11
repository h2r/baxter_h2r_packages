/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2.jse/src/org/jvoicexml/jsapi2/jse/AudioFormatConverter.java $
 * Version: $LastChangedRevision: 332 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An base implementation for JSR 113.
 *
 * Copyright (C) 2007-2009 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */

package org.jvoicexml.jsapi2.jse;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PipedInputStream;
import java.io.PipedOutputStream;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;

import org.jvoicexml.jsapi2.BaseAudioManager;

/**
 * Utility to convert audio from a source format to another format.
 * @author Renato Cassaca
 * @author Dirk Schnelle-Walka
 *
 */
public class AudioFormatConverter {

    /** The source audio format. */
    private final AudioFormat sourceFormat;
    /** The target audio format. */
    private final AudioFormat targetFormat;

    public AudioFormatConverter(BaseAudioManager manager,
            AudioFormat sourceFormat, AudioFormat targetFormat)
        throws IOException {
        this.sourceFormat = sourceFormat;
        this.targetFormat = targetFormat;
    }

    /**
     * Convert the given audio data from the source format to the target format.
     * @param in bytes to convert.
     * @return converted bytes stream
     * @throws IOException
     *         error converting
     */
    public InputStream getConvertedAudio(final byte[] in)
        throws IOException {
        final ByteArrayInputStream stream = new ByteArrayInputStream(in);
        return getConvertedStream(stream, sourceFormat, targetFormat);
    }

    /**
     * @todo Insure that this is robust...
     *
     * @param is InputStream
     * @param sourceFormat AudioFormat
     * @param targetFormat AudioFormat
     * @return InputStream
     */
    public InputStream getConvertedStream(final InputStream is,
            final AudioFormat sourceFormat, final AudioFormat targetFormat) {
        /** @todo Compare more precisely AudioFormat (not using AudioFormat.matches()) */
        if (sourceFormat.matches(targetFormat)) {
            return is;
        }

        // Describe source stream as an AudioFormat
        final AudioInputStream sourceStream = new AudioInputStream(is,
                sourceFormat, AudioSystem.NOT_SPECIFIED);
        return AudioSystem.getAudioInputStream(targetFormat, sourceStream);
    }

    /**
     * @todo Insure that this is robust...
     *
     * @param os OutputStream
     * @param engineAudioFormat AudioFormat
     * @param audioFormat AudioFormat
     * @return OutputStream
     */
    public OutputStream getConvertedStream(final OutputStream os,
                                            AudioFormat engineAudioFormat,
                                            AudioFormat targetFormat) {
        /** @todo Compare more preciselly AudioFormat (not using AudioFormat.matches()) */
        if (engineAudioFormat.matches(targetFormat)) {
            return os;
        }

        try {
            //Basic Conversion support
            PipedInputStream pis = new PipedInputStream(16000000);
            PipedOutputStream pos = new PipedOutputStream(pis);

            //Describe source audio
            getConvertedStream(pis, engineAudioFormat, targetFormat);

            return pos;
        } catch (IOException ex) {
            ex.printStackTrace();
        }

        /** @todo Should never reach this point */
        return os;
    }
}
