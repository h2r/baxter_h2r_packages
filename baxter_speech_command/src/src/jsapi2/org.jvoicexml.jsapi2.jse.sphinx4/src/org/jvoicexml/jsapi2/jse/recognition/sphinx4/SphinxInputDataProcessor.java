package org.jvoicexml.jsapi2.jse.recognition.sphinx4;

import java.io.IOException;
import java.io.InputStream;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.cmu.sphinx.frontend.BaseDataProcessor;
import edu.cmu.sphinx.frontend.Data;
import edu.cmu.sphinx.frontend.DataEndSignal;
import edu.cmu.sphinx.frontend.DataProcessingException;
import edu.cmu.sphinx.frontend.DataStartSignal;
import edu.cmu.sphinx.frontend.DoubleData;
import edu.cmu.sphinx.frontend.util.DataUtil;
import edu.cmu.sphinx.util.props.Configurable;

/**
 * <p>
 * Title: JSAPI2Engines
 * </p>
 * 
 * <p>
 * Description: JSAPI 2.0 Engines implementations
 * </p>
 * 
 * <p>
 * Copyright: Copyright (c) 2007
 * </p>
 * 
 * <p>
 * Company: INESC-ID L2F
 * </p>
 * 
 * @author Renato Cassaca
 * @author Stefan Radomski
 * @version 1.0
 */
public class SphinxInputDataProcessor extends BaseDataProcessor
        implements Configurable {
    /** Logger for this class. */
    private static final Logger LOGGER = Logger
            .getLogger(SphinxInputDataProcessor.class.getName());

    private InputStream inputStream;
    private long totalSamplesRead = 0;

    // sent DataStart signal
    private boolean sentStarted = false;

    // sent DataEnd signal
    private boolean sentEnded = false;

    // is running
    private boolean running = true;

    public SphinxInputDataProcessor() {
        super();
    }

    public void setInputStream(InputStream inputStream) {
        this.inputStream = inputStream;
    }

    /**
     * Start or stop this data processor
     * 
     * We need to send a DataStartSignal before commencing to send samples. When
     * we were stopped, send a DataEndSignal and return null for subsequent
     * calls to getData().
     * 
     * This way, we can stop this processor and have calls to recognize() return
     * immediately.
     */
    public synchronized void isRunning(boolean running) {
        if (this.running != running) {
            if (running)
                sentStarted = false;
            if (!running)
                sentEnded = false;
            this.running = running;
        }
    }

    /**
     * Returns the processed Data output.
     * 
     * @return an Data object that has been processed by this DataProcessor
     * @throws DataProcessingException
     *             if a data processor error occurs
     */
    public Data getData() throws DataProcessingException {

        /** @todo AudioFormat is hardcoded */
        int channels = 1;
        int sampleSizeInBytes = 2;
        boolean signed = true;
        int sampleRate = 16000;
        int frameSizeInBytes = sampleRate * sampleSizeInBytes * channels * 10
                / 1000;

        // we are not running anymore, but did not sent DataEnd yet
        if (!running && !sentEnded) {
            sentEnded = true;
            long duration = (long) (((double) totalSamplesRead
                    / (double) sampleRate * 1000.0));
            if (LOGGER.isLoggable(Level.FINE)) {
                LOGGER.fine("Sending end signal");
            }
            totalSamplesRead = 0;
            return new DataEndSignal(duration);
        }

        // we are running, but have not sent DataStart yet
        if (running && !sentStarted) {
            sentStarted = true;
            if (LOGGER.isLoggable(Level.FINE)) {
                LOGGER.fine("Sending start signal");
            }
            return new DataStartSignal(sampleRate);
        }

        // we are not running at all
        if (!running) {
            if (LOGGER.isLoggable(Level.FINE)) {
                LOGGER.fine("Sending null");
            }
            return null;
        }

        long collectTime = System.currentTimeMillis();
        long firstSampleNumber = totalSamplesRead / channels;

        // Read data
        byte[] data = new byte[frameSizeInBytes];
        int numBytesRead = 0;
        while (numBytesRead == 0) {
            try {
                numBytesRead = inputStream.read(data);
            } catch (IOException e) {
                throw new DataProcessingException(e.getMessage());
            }
            if (numBytesRead == 0) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new DataProcessingException(e.getMessage());
                }
            }
        }
        if (numBytesRead == -1) {
            long duration = (long) (((double) totalSamplesRead
                    / (double) sampleRate * 1000.0));

            return new DataEndSignal(duration);
        }

        if (numBytesRead != frameSizeInBytes) {
            if (numBytesRead % sampleSizeInBytes != 0) {
                // Is it an error?
                LOGGER.warning("Sphinx ReadData: Incomplete sample read.");
            }

            byte[] shrinked = new byte[numBytesRead];
            System.arraycopy(data, 0, shrinked, 0, numBytesRead);
            data = shrinked;
        }

        totalSamplesRead += (numBytesRead / sampleSizeInBytes);

        // Convert it to double
        double[] samples = DataUtil.bytesToValues(data, 0, data.length,
                sampleSizeInBytes, signed);

        return new DoubleData(samples, (int) sampleRate, collectTime,
                firstSampleNumber);
    }
}
