package org.jvoicexml.jsapi2;

import javax.speech.EngineProperties;

/**
 * Notification that a property in the {@link EngineProperties} should be
 * changed.
 * @author Dirk Schnelle-Walka
 *
 */
public class EnginePropertyChangeRequestEvent {
    private final EngineProperties source;
    private final String propertyName;
    private final Object oldValue;
    private final Object newValue;

    /**
     * Constructs a new object.
     * @param source the properties object.
     * @param propertyName name of the property
     * @param oldValue old value of the property
     * @param newValue new value of the property
     */
    public EnginePropertyChangeRequestEvent(final EngineProperties source,
            final String propertyName, final Object oldValue,
            final Object newValue) {
        if (source == null) {
            throw new IllegalArgumentException("Source must not be null!");
        }
        this.source = source;
        this.propertyName = propertyName;
        this.oldValue = oldValue;
        this.newValue = newValue;
        
    }

    public final EngineProperties getSource() {
        return source;
    }

    public final String getPropertyName() {
        return propertyName;
    }

    public final Object getOldValue() {
        return oldValue;
    }

    public final Object getNewValue() {
        return newValue;
    }
}
