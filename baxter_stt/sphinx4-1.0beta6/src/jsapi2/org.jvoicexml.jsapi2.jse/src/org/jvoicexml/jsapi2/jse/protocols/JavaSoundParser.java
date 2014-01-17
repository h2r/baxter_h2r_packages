/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2.jse/src/org/jvoicexml/jsapi2/jse/protocols/JavaSoundParser.java $
 * Version: $LastChangedRevision: 292 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An base implementation for JSR 113.
 *
 * Copyright (C) 2009 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */

package org.jvoicexml.jsapi2.jse.protocols;

import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;

import javax.sound.sampled.AudioFormat;

/**
 * A parser for the javasound URL schema.
 *
 * @author Dirk Schnelle-Walka
 *
 */
public final class JavaSoundParser {
    /** The default sample rate. */
    public static final int DEFAULT_SAMPLE_RATE = 8000;

    /** Number of bits per byte. */
    private static final int BITS_PER_BYTE = 8;

    /**
     * Prevent construction from outside.
     */
    private JavaSoundParser() {
    }

    /**
     * Parses the given URI into an audio format.
     *
     * @param url
     *            the URL to parse.
     * @return audio format.
     * @throws URISyntaxException
     *         error parsing the URL
     */
    public static AudioFormat parse(final URL url) throws URISyntaxException {
        URI uri = url.toURI();
        final Map<String, String> parameters = new HashMap<String, String>();
        if (uri.getQuery() != null) {
            String[] parametersString = uri.getQuery().split("\\&");
            for (String part : parametersString) {
                String[] queryElement = part.split("\\=");
                parameters.put(queryElement[0], queryElement[1]);
            }
        }

        // Default values for AudioFormat parameters
        AudioFormat.Encoding encoding = AudioFormat.Encoding.ULAW;
        float sampleRate = DEFAULT_SAMPLE_RATE;
        int bits = BITS_PER_BYTE;
        int channels = 1;
        boolean endian = true;
        boolean signed = true;

        // Change default values as specified
        final String signedStr = parameters.get("signed");
        if (signedStr != null) {
            signed = Boolean.valueOf(signedStr);
        }

        final String encodingStr = parameters.get("encoding");
        if (encodingStr != null) {
            if (encodingStr.equals("pcm")) {
                if (signed) {
                    encoding = AudioFormat.Encoding.PCM_SIGNED;
                } else {
                    encoding = AudioFormat.Encoding.PCM_UNSIGNED;
                }
            } else if (encodingStr.equals("alaw")) {
                encoding = AudioFormat.Encoding.ALAW;
            } else if (encodingStr.equals("ulaw")) {
                encoding = AudioFormat.Encoding.ULAW;
            } else if (encodingStr.equals("gsm")) {
                throw new URISyntaxException(url.toString(),
                        "gsm is currently not supported!");
            }
        }

        final String rateStr = parameters.get("rate");
        if (rateStr != null) {
            sampleRate = Float.valueOf(rateStr);
        }

        final String bitsStr = parameters.get("bits");
        if (bitsStr != null) {
            bits = Integer.valueOf(bitsStr);
        }

        final String channelsStr = parameters.get("channels");
        if (channelsStr != null) {
            channels = Integer.valueOf(channelsStr);
        }

        final String endianStr = parameters.get("endian");
        if (endianStr != null) {
            if (endianStr.equals("little")) {
                endian = false;
            } else if (endianStr.equals("big")) {
                endian = true;
            }
        }

        // Construct the AudioFormat
        return new AudioFormat(encoding, sampleRate,
                bits, channels, bits / BITS_PER_BYTE, sampleRate, endian);
    }
}
