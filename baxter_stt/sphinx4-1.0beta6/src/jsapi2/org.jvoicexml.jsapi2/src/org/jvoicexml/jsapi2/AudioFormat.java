package org.jvoicexml.jsapi2;

/**
 * Simple container for an audio format.
 * 
 * @author Dirk Schnelle-Walka
 */
public final class AudioFormat {
    /**
     * The audio encoding technique used by this format.
     */
    private String encoding;

    /**
     * The number of samples played or recorded per second, for sounds that have
     * this format.
     */
    private float sampleRate;

    /**
     * The number of bits in each sample of a sound that has this format.
     */
    private int sampleSizeInBits;

    /**
     * The number of audio channels in this format (1 for mono, 2 for stereo).
     */
    private int channels;

    /**
     * The number of bytes in each frame of a sound that has this format.
     */
    private int frameSize;

    /**
     * The number of frames played or recorded per second, for sounds that have
     * this format.
     */
    private float frameRate;

    /**
     * Indicates whether the audio data is stored in big-endian or little-endian
     * order.
     */
    private boolean bigEndian;

    /**
     * Constructs an <code>AudioFormat</code> with the given parameters.
     * 
     * @param enc
     *            the audio encoding technique
     * @param smpRate
     *            the number of samples per second
     * @param smpSize
     *            the number of bits in each sample
     * @param chan
     *            the number of channels (1 for mono, 2 for stereo, and so on)
     * @param frmSize
     *            the number of bytes in each frame
     * @param frmRate
     *            the number of frames per second
     * @param bigend
     *            indicates whether the data for a single sample is stored in
     *            big-endian byte order (<code>false</code> means little-endian)
     */
    public AudioFormat(final String enc, final float smpRate,
            final int smpSize, final int chan, final int frmSize,
            final float frmRate, final boolean bigend) {
        encoding = enc;
        sampleRate = smpRate;
        sampleSizeInBits = smpSize;
        channels = chan;
        frameSize = frmSize;
        frameRate = frmRate;
        bigEndian = bigend;
    }

    /**
     * Obtains the type of encoding for sounds in this format.
     * 
     * @return the encoding type
     */
    public String getEncoding() {
        return encoding;
    }

    /**
     * Obtains the sample rate.
     * 
     * @return the number of samples per second
     * 
     * @see #getFrameRate()
     */
    public float getSampleRate() {
        return sampleRate;
    }

    /**
     * Obtains the size of a sample.
     * 
     * @return the number of bits in each sample
     * 
     * @see #getFrameSize()
     */
    public int getSampleSizeInBits() {
        return sampleSizeInBits;
    }

    /**
     * Obtains the number of channels. 
     * 
     * @return The number of channels (1 for mono, 2 for stereo, etc.
     */
    public int getChannels() {
        return channels;
    }

    /**
     * Obtains the frame size in bytes.
     * 
     * @return the number of bytes per frame
     * 
     * @see #getSampleSizeInBits()
     */
    public int getFrameSize() {
        return frameSize;
    }

    /**
     * Obtains the frame rate in frames per second.
     * 
     * @return the number of frames per second
     * 
     * @see #getSampleRate()
     */
    public float getFrameRate() {
        return frameRate;
    }

    /**
     * Indicates whether the audio data is stored in big-endian or little-endian
     * byte order. If the sample size is not more than one byte, the return
     * value is irrelevant.
     * 
     * @return <code>true</code> if the data is stored in big-endian byte order,
     *         <code>false</code> if little-endian
     */
    public boolean isBigEndian() {

        return bigEndian;
    }
}
