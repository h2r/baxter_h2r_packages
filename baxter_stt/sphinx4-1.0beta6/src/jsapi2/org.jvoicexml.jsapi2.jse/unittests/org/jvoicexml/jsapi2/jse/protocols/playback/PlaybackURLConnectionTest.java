/**
 * 
 */
package org.jvoicexml.jsapi2.jse.protocols.playback;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.URL;
import java.net.UnknownServiceException;

import javax.sound.sampled.AudioSystem;

import org.junit.Before;
import org.junit.Test;

/**
 * Test cases for {@link PlaybackURLConnection}.
 * @author Dirk Schnelle-Walka
 *
 */
public class PlaybackURLConnectionTest {
    /**
     * Set up the test environment.
     */
    @Before
    public void setUp() {
        System.setProperty("java.protocol.handler.pkgs",
            "org.jvoicexml.jsapi2.jse.protocols");
    }

    /**
     * Test method for {@link net.sourceforge.gjtapi.protocols.PlaybackURLConnection#getInputStream()}.
     * @exception Exception
     *            test failed.
     */
    @Test(expected = UnknownServiceException.class)
    public void testGetInputStream() throws Exception {
        final URL url =
            new URL("capture://audio?rate=8000&channels=1&encoding=pcm");
        final PlaybackURLConnection connection = new PlaybackURLConnection(url);
        connection.connect();
        final InputStream input = connection.getInputStream();
    }

    /**
     * Test method for {@link net.sourceforge.gjtapi.protocols.PlaybackURLConnection#getOutputStream()}.
     * @exception Exception
     *            test failed.
     */
    @Test
    public void testGetOutputStream() throws Exception {
        System.out.println(AudioSystem.getClip().getFormat());
        final URL url =
            new URL("playback://audio?rate=44100&channels=2&encoding=pcm&bits=16");
        final PlaybackURLConnection connection = new PlaybackURLConnection(url);
        connection.connect();
        final OutputStream output = connection.getOutputStream();
    }

}
