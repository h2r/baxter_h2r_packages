package org.jvoicexml.jsapi2.recognition;

import java.util.Vector;
import javax.speech.recognition.SpeakerManager;
import javax.speech.recognition.SpeakerProfile;
import javax.speech.recognition.SpeakerManagerUI;
import javax.speech.EngineStateException;

/**
 * Basic implementation of a speaker manager.
 * @author Renato Cassaca
 * @author Dirk Schnelle-Walka
 */
public class BaseSpeakerManager implements SpeakerManager {

    private Vector speakerProfiles;

    private SpeakerProfile currentSpeaker;

    public BaseSpeakerManager() {
        speakerProfiles = new Vector();
        currentSpeaker = null;
    }

    /**
     * createSpeaker
     *
     * @param speaker SpeakerProfile
     */
    public void createSpeaker(final SpeakerProfile speaker) {
        speakerProfiles.addElement(speaker);
    }

    /**
     * deleteSpeaker
     *
     * @param speaker SpeakerProfile
     */
    public void deleteSpeaker(SpeakerProfile speaker) {
        speakerProfiles.removeElement(speaker);
    }

    /**
     * getCurrentSpeaker
     *
     * @return SpeakerProfile
     */
    public SpeakerProfile getCurrentSpeaker() {
        return currentSpeaker;
    }

    /**
     * getSpeakerManagerUI
     *
     * @return SpeakerManagerUI
     */
    public SpeakerManagerUI getSpeakerManagerUI() {
        return null;
    }

    /**
     * listKnownSpeakers
     *
     * @return SpeakerProfile[]
     */
    public SpeakerProfile[] listKnownSpeakers() {
        if (speakerProfiles.size() < 1) return new SpeakerProfile[]{};

        SpeakerProfile[] profiles = new SpeakerProfile[speakerProfiles.size()];
        for (int i = 0; i < speakerProfiles.size(); i++) {
            profiles[i] = (SpeakerProfile)speakerProfiles.elementAt(i);
        }
        return profiles;

    }

    /**
     * renameSpeaker
     *
     * @param oldSpeaker SpeakerProfile
     * @param newSpeaker SpeakerProfile
     */
    public void renameSpeaker(SpeakerProfile oldSpeaker,
                              SpeakerProfile newSpeaker) {
    }

    /**
     * restoreCurrentSpeaker
     *
     * @throws EngineStateException
     */
    public void restoreCurrentSpeaker() throws EngineStateException {
    }

    /**
     * saveCurrentSpeaker
     *
     * @throws EngineStateException
     */
    public void saveCurrentSpeaker() throws EngineStateException {
    }

    /**
     * setCurrentSpeaker
     *
     * @param speaker SpeakerProfile
     */
    public void setCurrentSpeaker(SpeakerProfile speaker) {
        if (speakerProfiles.contains(speaker) == false) {
            createSpeaker(speaker);
        }
        currentSpeaker = speaker;
    }
}
