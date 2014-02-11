package org.jvoicexml.jsapi2;

import java.io.InputStream;
import java.io.OutputStream;

import javax.speech.AudioException;

public class DummyAudioManager extends BaseAudioManager {

    public AudioFormat getAudioFormat() {
        return new AudioFormat("ulaw", 8000.0f, 16, 1, 16, 8000, false);
    }

    public InputStream getInputStream() {
        // TODO Auto-generated method stub
        return null;
    }

    public OutputStream getOutputStream() {
        // TODO Auto-generated method stub
        return null;
    }

    protected void handleAudioStart() throws AudioException {
        // TODO Auto-generated method stub

    }

    protected void handleAudioStop() throws AudioException {
        // TODO Auto-generated method stub

    }

    public void setMediaLocator(String locator, InputStream stream)
            throws AudioException, IllegalStateException,
            IllegalArgumentException, SecurityException {
        // TODO Auto-generated method stub

    }

    public void setMediaLocator(String locator, OutputStream stream)
            throws AudioException, IllegalStateException,
            IllegalArgumentException, SecurityException {
        // TODO Auto-generated method stub

    }

}
