package org.jvoicexml.jsapi2.jse.recognition;


import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.io.StringReader;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Vector;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.speech.EngineException;
import javax.speech.EngineMode;
import javax.speech.EngineStateException;
import javax.speech.SpeechLocale;
import javax.speech.recognition.Grammar;
import javax.speech.recognition.GrammarEvent;
import javax.speech.recognition.GrammarException;
import javax.speech.recognition.GrammarListener;
import javax.speech.recognition.GrammarManager;
import javax.speech.recognition.Recognizer;
import javax.speech.recognition.Rule;
import javax.speech.recognition.RuleGrammar;

/**
 * A base implementation of a {@link GrammarManager}.
 *
 * @author Renato Cassaca
 * @author Dirk Schnelle-Walka
 */
public class BaseGrammarManager implements GrammarManager {

    /** Logger for this class. */
    private static final Logger LOGGER =
            Logger.getLogger(BaseGrammarManager.class.getName());
    
    /** The listeners of grammar events. */
    protected final List<GrammarListener> grammarListeners;

    /** Storage of created grammars. */
    protected HashMap<String, Grammar> grammars; // final gone

    /** Mask that filter events. */
    private int grammarMask;

    /** Recognizer which the GrammarManager belongs. */
    private final JseBaseRecognizer recognizer;

    /**
     * Constructor that associates a Recognizer.
     * with a GrammarManager
     *
     * @param reco BaseRecognizer
     */
    public BaseGrammarManager(final JseBaseRecognizer reco) {
        grammarListeners = new ArrayList<GrammarListener>();
        grammars = new HashMap<String, Grammar>();
        grammarMask = GrammarEvent.DEFAULT_MASK;
        recognizer = reco;
    }

    /**
     * Constructor that allows to use a GrammarManager in
     * standalone mode.
     */
    public BaseGrammarManager() {
        this(null);
    }

    /**
     * {@inheritDoc}
     */
    public void addGrammarListener(final GrammarListener listener) {
        grammarListeners.add(listener);
    }

    /**
     * {@inheritDoc}
     */
    public void removeGrammarListener(final GrammarListener listener) {
        grammarListeners.remove(listener);
    }

    /**
     * {@inheritDoc}
     */
    public RuleGrammar createRuleGrammar(String grammarReference,
                                         String rootName) throws
            IllegalArgumentException, EngineStateException, EngineException {
        final SpeechLocale locale = SpeechLocale.getDefault();
        return createRuleGrammar(grammarReference, rootName, locale);
    }

    /**
     * {@inheritDoc}
     */
    public RuleGrammar createRuleGrammar(String grammarReference,
                                         String rootName,
                                         SpeechLocale locale) throws
            IllegalArgumentException, EngineStateException, EngineException {

        //Validate current state
        ensureValidEngineState();

        if (grammars.containsValue(grammarReference)) {
            throw new IllegalArgumentException("Duplicate grammar name: " +
                                               grammarReference);
        }

        //Create grammar
        final BaseRuleGrammar brg =
            new BaseRuleGrammar(recognizer, grammarReference);
        brg.setAttribute("xml:lang", locale.toString());
        brg.setRoot("test");

        //Register it
        grammars.put(grammarReference, brg);

        return brg;
    }

    /**
     * Deletes a Grammar
     *
     * @param grammar Grammar
     * @throws IllegalArgumentException
     * @throws EngineStateException
     */
    public void deleteGrammar(Grammar grammar) throws IllegalArgumentException,
            EngineStateException {

        //Validate current state
        ensureValidEngineState();

        if (!grammars.containsKey(grammar.getReference()))
            throw new IllegalArgumentException("The Grammar is unknown");

        //Remove the grammar
        Grammar key = grammars.remove(grammar.getReference());
        
        
        if(LOGGER.isLoggable(Level.FINE)){
            LOGGER.fine("Removed Grammar :"+ key.getReference());
            Iterator<String> keys = grammars.keySet().iterator();
            
            while (keys.hasNext()){
                LOGGER.fine("Grammar :"+ keys.next() );
            }
        }
        
        
    }

    /**
     * Lists the Grammars known to this Recognizer
     *
     * @return Grammar[]
     * @throws EngineStateException
     */
    public Grammar[] listGrammars() throws EngineStateException {

        // Validate current state
        ensureValidEngineState();

        // List of all grammars
        ArrayList<Grammar> allGrammars = new ArrayList<Grammar> ();

        // Get engine built-in grammars
        if (recognizer != null) {
            Vector builtInGrammars = recognizer.getBuiltInGrammars();
            if (builtInGrammars != null) {
                allGrammars.addAll(builtInGrammars);
            }
        }

        // Add local managed grammars
        allGrammars.addAll(grammars.values());

        // Return an array with all know grammars
        return (Grammar[]) allGrammars.toArray(new Grammar[allGrammars.size()]);
    }

    /**
     * Gets the RuleGrammar with the specified grammarReference.
     *
     * @param grammarReference String
     * @return RuleGrammar
     * @throws EngineStateException
     */
    public Grammar getGrammar(String grammarReference) throws
            EngineStateException {

        // Validate current state
        ensureValidEngineState();

        return grammars.get(grammarReference);
    }

    /**
     * Loads a RuleGrammar from a URI or named resource.
     *
     * @param grammarReference String
     * @return RuleGrammar
     * @throws GrammarException
     * @throws IllegalArgumentException
     * @throws IOException
     * @throws EngineStateException
     * @throws EngineException
     */
    public Grammar loadGrammar(String grammarReference, String mediaType) throws
            GrammarException, IllegalArgumentException, IOException,
            EngineStateException, EngineException {
        
        if(LOGGER.isLoggable(Level.FINE)){
            LOGGER.fine("Load Grammar : "+ grammarReference + " with media Type:"+ mediaType);
        }
               
        return loadGrammar(grammarReference, mediaType, true, false, null);
    }

    /**
     * Loads a RuleGrammar from a URI or named resource
     * and optionally loads any referenced Grammars.
     *
     * @param grammarReference String
     * @param loadReferences boolean
     * @param reloadGrammars boolean
     * @param loadedGrammars Vector
     * @return RuleGrammar
     * @throws GrammarException
     * @throws IllegalArgumentException
     * @throws IOException
     * @throws EngineStateException
     * @throws EngineException
     */
    public Grammar loadGrammar(String grammarReference,
                               String mediaType,
                               boolean loadReferences,
                               boolean reloadGrammars,
                               Vector loadedGrammars) throws
            GrammarException, IllegalArgumentException,
            IOException, EngineStateException, EngineException {
        
        if(LOGGER.isLoggable(Level.FINE)){
            LOGGER.fine("Load Grammar : "+ grammarReference + " with media Type:"+ mediaType);
            LOGGER.fine("loadReferences : "+ loadReferences + " reloadGrammars:"+ reloadGrammars);
            LOGGER.fine("there are "+ loadedGrammars.size() +" loaded grammars:" );
        }
        
        //Validate current state
        ensureValidEngineState();

        //Make sure that recognizer supports markup
        if (recognizer != null) {
            final EngineMode mode = recognizer.getEngineMode();
            if (!mode.getSupportsMarkup()) {
                throw new EngineException("Engine doesn't support markup");
            }
        }

        //Proccess grammar
        URL url = new URL(grammarReference);
        InputStream grammarStream = url.openStream();
        
        SrgsRuleGrammarParser srgsParser = new SrgsRuleGrammarParser();
        Rule[] rules = srgsParser.load(grammarStream);
        if (rules != null) {
            //Initialize rule grammar
            BaseRuleGrammar brg = new BaseRuleGrammar(recognizer,
                    grammarReference);
            brg.addRules(rules);
            brg.setAttributes(srgsParser.getAttributes());

            //Register grammar
            grammars.put(grammarReference, brg);

            return brg;
        }

        return null;
    }

    /**
     * Creates a RuleGrammar from grammar text provided by a Reader.
     *
     * @param grammarReference String
     * @param reader Reader
     * @return RuleGrammar
     * @throws GrammarException
     * @throws IllegalArgumentException
     * @throws IOException
     * @throws EngineStateException
     * @throws EngineException
     */
    @SuppressWarnings("unchecked")
    public Grammar loadGrammar(String grammarReference, String mediaType,
                               Reader reader) throws
            GrammarException, IllegalArgumentException, IOException,
            EngineStateException, EngineException {
        
        if(LOGGER.isLoggable(Level.FINE)){
            LOGGER.fine("Load Grammar : "+ grammarReference + " with media Type:"+ mediaType + " and Reader :"+ reader);
        }
        // Validate current state
        ensureValidEngineState();

        // Make sure that recognizer supports markup
        if (recognizer != null) {
            final EngineMode mode = recognizer.getEngineMode();
            if (!mode.getSupportsMarkup()) {
                throw new EngineException("Engine doesn't support markup");
            }
        }

        // Process grammar
        final SrgsRuleGrammarParser srgsParser = new SrgsRuleGrammarParser(); 

        Rule[] rules = srgsParser.load(reader);
        if (rules == null) {
            return null;
        } 
        
        if(LOGGER.isLoggable(Level.FINE)){
            LOGGER.fine("SrgsRuleGrammarParser parsed rules:" );
            for(Rule rule: rules){               
                LOGGER.fine( rule.getRuleName() );
            }           
        }
        
        // Initialize rule grammar
        BaseRuleGrammar brg =
            new BaseRuleGrammar(recognizer, grammarReference);
        
        if(LOGGER.isLoggable(Level.FINE)){
            LOGGER.fine("new BaseRuleGrammar:"+ brg.getReference() );       
        }       
       
        brg.addRules(rules);
        
        @SuppressWarnings("rawtypes")       
        final HashMap attributes = srgsParser.getAttributes();
        
        if(LOGGER.isLoggable(Level.FINE)){
            LOGGER.fine("recieved from srgsParser.getAttributes(), root:"+ attributes.get("root") );       
        }   
        
//        attributes.remove("root");
//        String[] value = grammarReference.split(":");
//        
//        attributes.put("root", value[1].trim() );
//        if(LOGGER.isDebugEnabled()){
//            LOGGER.debug("replaced root with:" + value[1]);       
//        }     
//        
        brg.setAttributes(attributes);

        // Register grammar
        grammars.put(grammarReference, brg);

        return brg;
    }

    /**
     * Creates a RuleGrammar from grammar text provided as a String.
     *
     * @param grammarReference String
     * @param grammarText String
     * @return RuleGrammar
     * @throws GrammarException
     * @throws IllegalArgumentException
     * @throws IOException
     * @throws EngineStateException
     * @throws EngineException
     */
    public Grammar loadGrammar(String grammarReference,
                               String mediaType,
                               String grammarText) throws
            GrammarException, IllegalArgumentException, IOException,
            EngineStateException, EngineException {
        return loadGrammar(grammarReference, mediaType,
                           new StringReader(grammarText));
    }

    /**
     *
     * @param grammarReference String
     * @param mediaType String
     * @param byteStream InputStream
     * @param encoding String
     * @return Grammar
     * @throws GrammarException
     * @throws IllegalArgumentException
     * @throws IOException
     * @throws EngineStateException
     * @throws EngineException
     */
    public Grammar loadGrammar(String grammarReference,
                               String mediaType,
                               InputStream byteStream,
                               String encoding) throws GrammarException,
            IllegalArgumentException,
            IOException,
            EngineStateException,
            EngineException {

        final InputStreamReader reader =
            new InputStreamReader(byteStream, encoding);
        return loadGrammar(grammarReference, mediaType, reader);
    }


    public void setGrammarMask(int mask) {
        grammarMask = mask;
    }

    public int getGrammarMask() {
        return grammarMask;
    }

    /**
     * Checks if the recognizer is in a valid state to perform grammar
     * operations. If the recognizer is currently allocating resources, this
     * methods waits until the the resoucres are allocated.
     * @throws EngineStateException
     *         invalid engine state
     */
    private void ensureValidEngineState() throws EngineStateException {
        if (recognizer == null) {
            return;
        }
        //Validate current state
        if (recognizer.testEngineState(
                Recognizer.DEALLOCATED | Recognizer.DEALLOCATING_RESOURCES)) {
            throw new EngineStateException(
              "Cannot execute GrammarManager operation: invalid engine state: "
              + recognizer.stateToString(recognizer.getEngineState()));
        }

        //Wait until end of allocating (if it's currently allocating)
        while (recognizer.testEngineState(Recognizer.ALLOCATING_RESOURCES)) {
            try {
                recognizer.waitEngineState(Recognizer.ALLOCATED);
            } catch (InterruptedException ex) {
                throw new EngineStateException(ex.getMessage());
            }
        }
    }
}
