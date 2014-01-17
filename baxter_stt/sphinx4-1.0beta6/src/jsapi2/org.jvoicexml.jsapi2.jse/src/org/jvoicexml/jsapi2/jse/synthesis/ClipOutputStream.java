/**
 * 
 */
package org.jvoicexml.jsapi2.jse.synthesis;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.LineEvent;
import javax.sound.sampled.LineListener;
import javax.sound.sampled.LineUnavailableException;

import org.jvoicexml.jsapi2.jse.JseBaseAudioManager;

/**
 * A simple {@link OutputStream} that writes the data to the local speaker.
 *
 * @author Dirk Schnelle-Walka
 *
 */
public class ClipOutputStream extends OutputStream implements LineListener {
    /** The audio buffer. */
    private ByteArrayOutputStream buffer;

    /** Synchronization of start and end play back. */
    private final Object lock;

    /** The audio manager to use. */
    private final JseBaseAudioManager manager;

    /** The current clip. */
    private Clip clip;

    /**
     * Constructs a new object.
     * @param audioManager the audio manger
     */
    public ClipOutputStream(final JseBaseAudioManager audioManager) {
        buffer = new ByteArrayOutputStream();
        lock = new Object();
        manager = audioManager;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void write(final int b) throws IOException {
        synchronized (buffer) {
            buffer.write(b);
        }
    }


    /**
     * {@inheritDoc}
     */
    @Override
    public void write(final byte[] b, final int off, final int len)
        throws IOException {
        synchronized (buffer) {
            buffer.write(b, off, len);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void write(final byte[] b) throws IOException {
        synchronized (buffer) {
            buffer.write(b);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void flush() throws IOException {
        final AudioFormat format = manager.getTargetAudioFormat();
        try {
            final DataLine.Info info = new DataLine.Info(Clip.class, format);
            clip = (Clip) AudioSystem.getLine(info);
            clip.addLineListener(this);
            final byte[] bytes;
            synchronized (buffer) {
                bytes = buffer.toByteArray();
                buffer.reset();
            }
            clip.open(format, bytes, 0, bytes.length);
            clip.start();
        } catch (LineUnavailableException e) {
            clip = null;
            throw new IOException(e.getMessage(), e);
        }

        // Wait until all data has been played back.
        try {
            synchronized (lock) {
                lock.wait();
            }
            clip.removeLineListener(this);
            clip.stop();
            clip.close();
        } catch (InterruptedException e) {
            throw new IOException(e.getMessage(), e);
        } finally {
            clip = null;
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void close() throws IOException {
        if (clip != null) {
            clip.stop();
            clip.close();
        }
        super.close();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void update(final LineEvent event) {
        if ((event.getType() == LineEvent.Type.CLOSE)
                || (event.getType() == LineEvent.Type.STOP)) {
            synchronized (lock) {
                lock.notifyAll();
            }
        }
    }
}
