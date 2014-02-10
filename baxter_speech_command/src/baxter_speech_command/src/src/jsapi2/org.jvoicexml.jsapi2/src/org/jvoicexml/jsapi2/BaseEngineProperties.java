/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2/src/org/jvoicexml/jsapi2/BaseEngineProperties.java $
 * Version: $LastChangedRevision: 295 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An independent reference implementation of JSR 113.
 *
 * Copyright (C) 2007-2009 JVoiceXML group - http://jvoicexml.sourceforge.net
 */

package org.jvoicexml.jsapi2;

import java.util.Enumeration;
import java.util.Vector;

import javax.speech.Engine;
import javax.speech.EngineProperties;
import javax.speech.EnginePropertyEvent;
import javax.speech.EnginePropertyListener;
import javax.speech.SpeechEventExecutor;

/**
 * Supports the JSAPI 2.0 <code>EngineProperties</code> interface.
 * 
 * <p>
 * Actual JSAPI 2 implementations may want to override this class to apply the
 * settings to the engine.
 * </p>
 *
 * <p>
 * Engines are requested to implement the
 * {@link EnginePropertyChangeRequestListener} interface and register
 * to this object via the
 * {@link #addEnginePropertyChangeRequestListener(EnginePropertyChangeRequestListener)}
 * method to get notifications about change requests-
 * Properties are not set directly. Instead the
 * {@link EnginePropertyChangeRequestListener} is notified about a change
 * request.
 * Once a requested property
 * is changed the engine must call the callback method
 * {@link #commitPropertyChange(String, Object, Object)} to have all
 * registered {@link EnginePropertyListener}s notified about the change.
 * </p>
 *
 * @author Renato Cassaca
 * @author Dirk Schnelle-Walka
 * @version $Revision: 295 $
 */
public abstract class BaseEngineProperties implements EngineProperties {
    /** Name of the base property in events. */
    public static final String BASE = "base";

    /** Name of the priority property in events. */
    public static final String PRIORITY = "priority";

    /**
     * List of <code>PropertyChangeListeners</code> registered for
     * <code>PropertyChangeEvents</code> on this object.
     */
    private final Vector propertyChangeListeners;

    /**
     * List of {@link EnginePropertyChangeRequestListener} registered for
     * {@link EnginePropertyChangeRequestEvent} on this object.
     */
    private final Vector propertyChangeRequestListeners;

    /** The engine for which these properties apply. */
    private final Engine engine;

    /**
     * The base location to resolve relative URLs against for this Engine
     * instance.
     */
    private String base;

    /** The priority for this engine instance. */
    private int priority;

    /**
     * Constructs a new object.
     * 
     * @param eng
     *            the engine for which these properties apply.
     */
    protected BaseEngineProperties(final Engine eng) {
        propertyChangeListeners = new Vector();
        propertyChangeRequestListeners = new Vector();
        engine = eng;
        priority = EngineProperties.NORM_TRUSTED_PRIORITY;
        base = "";
    }

    /**
     * Retrieves the engine.
     * 
     * @return the engine
     */
    protected final Engine getEngine() {
        return engine;
    }

    /**
     * Returns all properties to reasonable defaults for the <code>Engine</code>
     * . A <code>PropertyChangeEvent</code> is issued for each property that
     * changes as the reset takes effect.
     */
    public void reset() {
        setPriority(EngineProperties.NORM_TRUSTED_PRIORITY);
        setBase("");
    }

    /**
     * {@inheritDoc}
     */
    public int getPriority() {
        return priority;
    }

    /**
     * {@inheritDoc}
     */
    public void setPriority(final int prio) {
        postPropertyChangeRequestEvent(PRIORITY, new Integer(this.priority),
                new Integer(prio));
    }

    /**
     * {@inheritDoc}
     */
    public void setBase(final String uri) {
        postPropertyChangeRequestEvent(BASE, new Integer(this.priority),
                new Integer(priority));
    }

    /**
     * {@inheritDoc}
     */
    public String getBase() {
        return base;
    }

    /**
     * {@inheritDoc}
     */
    public void addEnginePropertyListener(
            final EnginePropertyListener listener) {
        if (!propertyChangeListeners.contains(listener)) {
            propertyChangeListeners.addElement(listener);
        }
    }

    /**
     * {@inheritDoc}
     */
    public void removeEnginePropertyListener(
            final EnginePropertyListener listener) {
        propertyChangeListeners.removeElement(listener);
    }

    /**
     * Adds the given {@link EnginePropertyChangeRequestListener} to the list of
     * known listeners.
     * 
     * @param listener
     *            the listener to add
     */
    public void addEnginePropertyChangeRequestListener(
            final EnginePropertyChangeRequestListener listener) {
        if (!propertyChangeRequestListeners.contains(listener)) {
            propertyChangeRequestListeners.addElement(listener);
        }
    }

    /**
     * {@inheritDoc}
     */
    public void removeEnginePropertyChangeRequestListener(
            final EnginePropertyChangeRequestListener listener) {
        propertyChangeRequestListeners.removeElement(listener);
    }

    /**
     * Commit the property changes and sets the value.
     * <p>
     * This method is called after the new values are applied to the engine.
     * </p>
     * @param propName
     * @param oldValue
     * @param newValue
     */
    public void commitPropertyChange(final String propName,
            final Object oldValue, final Object newValue) {
        if (propName == null) {
            throw new IllegalArgumentException(
                    "Property name must not be null!");
        }
        if (propName.equals(PRIORITY)) {
            final Integer value = (Integer) newValue;
            priority = value.intValue();
        } else if (propName.equals("base")) {
            base = (String) newValue;
        } else {
            final boolean set = setProperty(propName, newValue);
            if (!set) {
                throw new IllegalArgumentException("Unknown property '"
                        + propName + "'");
            }
        }
        postPropertyChangeEvent(propName, oldValue, newValue);
    }

    /**
     * Must be overwritten to set engine specific properties which are not
     * covered by the standard after the values are applied to the engine.
     * @param propName name of the property
     * @param value value to set
     * @return <code>true</code> if the value has been set.
     */
    protected abstract boolean setProperty(final String propName,
            final Object value);

    /**
     * Synchronously notifies the registered listeners about the property change
     * request.
     * 
     * @param propName
     *            name of the property
     * @param oldValue
     *            old value of the property
     * @param newValue
     *            requested value of the property
     */
    protected void postPropertyChangeRequestEvent(final String propName,
            final Object oldValue, final Object newValue) {
        final EnginePropertyChangeRequestEvent event =
            new EnginePropertyChangeRequestEvent(
                this, propName, oldValue, newValue);
        final Enumeration e = propertyChangeRequestListeners.elements();
        while (e.hasMoreElements()) {
            final EnginePropertyChangeRequestListener listener
                = (EnginePropertyChangeRequestListener) e.nextElement();
            listener.propertyChangeRequest(event);
        }
    }

    /**
     * Generates a {@link PropertyChangeEvent} for an <code>Object</code> value
     * and posts it to the event queue using the configured
     * {@link SpeechEventExecutor}.
     * 
     * <p>
     * Registered listeners are notified using the
     * {@link #firePropertyChangeEvent(PropertyChangeEvent)} method.
     * </p>
     * 
     * @param propName
     *            the name of the property
     * @param oldValue
     *            the old value
     * @param newValue
     *            the new value
     * 
     * @see #firePropertyChangeEvent
     * @see #dispatchSpeechEvent
     */
    protected void postPropertyChangeEvent(final String propName,
            final Object oldValue, final Object newValue) {

        if (propertyChangeListeners.size() < 1) {
            return;
        }

        final EnginePropertyEvent event = new EnginePropertyEvent(this,
                propName, oldValue, newValue);

        final Runnable runnable = new Runnable() {
            public void run() {
                firePropertyChangeEvent(event);
            }
        };

        final SpeechEventExecutor executor = engine.getSpeechEventExecutor();
        executor.execute(runnable);
    }

    /**
     * Sends a {@link PropertyChangeEvent} to all
     * <code>PropertyChangeListeners</code> registered with this object.
     * 
     * <p>
     * This method runs within the configured {@link SpeechEventExecutor}.
     * </p>
     * 
     * @param event
     *            the <code>PropertyChangeEvent</code> to send
     * 
     * @see #firePropertyChangeEvent
     * @see #dispatchSpeechEvent
     */
    public void firePropertyChangeEvent(final EnginePropertyEvent event) {
        final Enumeration e = propertyChangeListeners.elements();
        while (e.hasMoreElements()) {
            final EnginePropertyListener listener = (EnginePropertyListener) e
                    .nextElement();
            listener.propertyUpdate(event);
        }
    }
}
