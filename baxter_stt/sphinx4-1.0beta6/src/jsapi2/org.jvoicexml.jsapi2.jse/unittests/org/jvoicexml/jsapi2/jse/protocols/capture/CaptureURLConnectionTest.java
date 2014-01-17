/**
 * 
 */
package org.jvoicexml.jsapi2.jse.protocols.capture;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.URL;
import java.net.UnknownServiceException;

import org.junit.Before;
import org.junit.Test;

/**
 * Test cases for {@link CaptureURLConnection}.
 * @author Dirk Schnelle-Walka
 *
 */
public class CaptureURLConnectionTest {
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
    @Test
    public void testGetInputStream() throws Exception {
        final URL url =
            new URL("capture://audio?rate=8000&channels=1&encoding=pcm");
        final CaptureURLConnection connection = new CaptureURLConnection(url);
        connection.connect();
        final InputStream input = connection.getInputStream();
    }

    /**
     * Test method for {@link net.sourceforge.gjtapi.protocols.PlaybackURLConnection#getOutputStream()}.
     * @exception Exception
     *            test failed.
     */
    @Test(expected = UnknownServiceException.class)
    public void testGetOutputStream() throws Exception {
        final URL url =
            new URL("playback://audio?rate=8000&channels=1&encoding=pcm");
        final CaptureURLConnection connection = new CaptureURLConnection(url);
        connection.connect();
        final OutputStream output = connection.getOutputStream();
    }

}
