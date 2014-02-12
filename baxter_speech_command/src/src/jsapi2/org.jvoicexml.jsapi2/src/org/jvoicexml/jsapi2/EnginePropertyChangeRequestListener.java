package org.jvoicexml.jsapi2;

/**
 * Listener to change requests in the {@link javax.speech.EngineProperties}.
 * @author Dirk Schnelle-Walka
 *
 */
public interface EnginePropertyChangeRequestListener {
    /**
     * A change request has been made.
     * @param event the notification about the change request
     */
    void propertyChangeRequest(final EnginePropertyChangeRequestEvent event);
}
