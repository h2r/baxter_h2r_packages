/*
 * Copyright 1999-2002 Carnegie Mellon University.
 * Portions Copyright 2002 Sun Microsystems, Inc.
 * Portions Copyright 2002 Mitsubishi Electric Research Laboratories.
 * All Rights Reserved.  Use is subject to license terms.
 *
 * See the file "license.terms" for information on usage and
 * redistribution of this file, and for a DISCLAIMER OF ALL
 * WARRANTIES.
 *
 */

package org.jvoicexml.jsapi2.jse.recognition.sphinx4;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.speech.EngineStateException;
import javax.speech.recognition.GrammarException;
import javax.speech.recognition.GrammarExceptionDetail;
import javax.speech.recognition.Rule;
import javax.speech.recognition.RuleAlternatives;
import javax.speech.recognition.RuleComponent;
import javax.speech.recognition.RuleCount;
import javax.speech.recognition.RuleGrammar;
import javax.speech.recognition.RuleParse;
import javax.speech.recognition.RuleReference;
import javax.speech.recognition.RuleSequence;
import javax.speech.recognition.RuleSpecial;
import javax.speech.recognition.RuleTag;
import javax.speech.recognition.RuleToken;

import org.jvoicexml.jsapi2.jse.recognition.BaseRuleGrammar;
import org.jvoicexml.jsapi2.jse.recognition.SrgsRuleGrammarParser;

import edu.cmu.sphinx.linguist.dictionary.Dictionary;
import edu.cmu.sphinx.linguist.language.grammar.Grammar;
import edu.cmu.sphinx.linguist.language.grammar.GrammarNode;
import edu.cmu.sphinx.util.LogMath;
import edu.cmu.sphinx.util.props.ConfigurationManagerUtils;
import edu.cmu.sphinx.util.props.PropertyException;
import edu.cmu.sphinx.util.props.PropertySheet;
import edu.cmu.sphinx.util.props.S4Component;
import edu.cmu.sphinx.util.props.S4String;

/**
 * Defines a XML-style grammar based on SRGS grammar rules in a file.
 * <p/>
 * <p/>
 * The Speech Recognition Grammar Specification (SRGS) is a XML-style,
 * platform-independent, and vendor-independent textual representation of
 * grammars for use in speech recognition.
 * <p/>
 * <p/>
 * You can quickly learn to write your own grammars. For analyse a complete
 * specification of SRGS, go to
 * <p>
 * <a href="http://www.w3.org/TR/speech-grammar/">http://www.w3.org/TR/speech-
 * grammar/</a>.
 * <p/>
 * <p/>
 * <h3>From SRGS to Grammar Graph</h3>
 * <p/>
 * After the SRGS grammar is read in, it is converted to a graph of words
 * representing the grammar. Lets call this the grammar graph. It is from this
 * grammar graph that the eventual search structure used for speech recognition
 * is built. Below, we show the grammar graphs created from the above SRGS
 * grammars. The nodes <code>"&lt;sil&gt;"</code> means "silence".
 * <p/>
 * <p/>
 * <h3>Limitations</h3>
 * <p/>
 * There are some known limitations with the current SRGS support: RuleTag,
 * RuleSpecial and RuleLocale.
 * <p/>
 * <h3>Implementation Notes</h3>
 * <ol>
 * <li>All internal probabilities are maintained in LogMath log base.
 * </ol>
 */

public class SRGSGrammar extends Grammar {
    /** Logger for this class. */
    private static final Logger LOGGER = Logger
            .getLogger(Sphinx4Recognizer.class.getName());

    /** Sphinx property that defines the location of the JSGF grammar file. */
    @S4String(defaultValue = "")
    public final static String PROP_BASE_GRAMMAR_URL = "grammarLocation";

    /** Sphinx property that defines the location of the JSGF grammar file. */
    @S4String(defaultValue = "default.gram")
    public final static String PROP_GRAMMAR_NAME = "grammarName";

    /** Sphinx property that defines the logMath component. */
    @S4Component(type = LogMath.class)
    public final static String PROP_LOG_MATH = "logMath";

    // ---------------------
    // Configurable data
    // ---------------------
    private BaseRuleGrammar ruleGrammar;
    private RuleStack ruleStack;
    private String grammarName;
    private URL baseURL = null;
    private URL grammarURL = null;
    private String grammarSTR;
    private LogMath logMath;

    private boolean loadGrammar = true;
    private GrammarNode firstNode = null;

    /**
     * The sphinx4 ConfigurationManager cannot get a newInstance() if a
     * constructor with formal parameters, but no default constructor is
     * defined.
     */
    public SRGSGrammar() {
        super();
    }

    public SRGSGrammar(boolean showGrammar, boolean optimizeGrammar,
            boolean addSilenceWords, boolean addFillerWords,
            Dictionary dictionary) {
        super(showGrammar, optimizeGrammar, addSilenceWords, addFillerWords,
                dictionary);
    }

    /*
     * (non-Javadoc)
     * 
     * @see
     * edu.cmu.sphinx.util.props.Configurable#newProperties(edu.cmu.sphinx.util
     * .props.PropertySheet)
     */
    public void newProperties(PropertySheet ps) throws PropertyException {
        super.newProperties(ps);
        baseURL = ConfigurationManagerUtils.getResource(PROP_BASE_GRAMMAR_URL,
                ps);
        logMath = (LogMath) ps.getComponent(PROP_LOG_MATH);

        grammarName = ps.getString(PROP_GRAMMAR_NAME);

        try {
            grammarURL = new URL(baseURL.toString() + grammarName);
            grammarURL.openStream();
        } catch (MalformedURLException ex) {
            ex.printStackTrace();
        } catch (IOException e) {
            // there is no grammar at the given location or no location is given
            return;
        }

        loadGrammar = true;
    }

    /**
     * Returns the RuleGrammar of this SRGSGrammar.
     * 
     * @return the RuleGrammar
     */
    public RuleGrammar getRuleGrammar() {
        return ruleGrammar;
    }

    /**
     * Sets the URL context of the SRGS grammars.
     * 
     * @param url
     *            the URL context of the grammars
     */
    public void setBaseURL(URL url) {
        baseURL = url;
    }

    public void setGrammarName(String newName) {
        grammarName = newName;
    }

    /** Returns the name of this grammar. */
    public String getGrammarName() {
        return grammarName;
    }

    /**
     * The SRGS grammar specified by grammarName will be loaded from the base
     * url (tossing out any previously loaded grammars)
     * 
     * @param grammarName
     *            the name of the grammar
     * @throws IOException
     *             if an error occurs while loading or compiling the grammar
     */
    public void loadSRGS(URL grammarURL) throws IOException {
        this.grammarURL = grammarURL;
        loadGrammar = true;
        commitChanges();
    }

    public void loadSRGS(String grammar) throws IOException {
        this.grammarURL = null;
        grammarSTR = grammar;
        loadGrammar = true;
        commitChanges();
    }

    /**
     * Creates the grammar.
     * 
     * @return the initial node of the Grammar
     */
    protected GrammarNode createGrammar() throws IOException {
        commitChanges();
        return firstNode;
    }

    /**
     * Returns the initial node for the grammar
     * 
     * @return the initial grammar node
     */
    public GrammarNode getInitialNode() {
        return firstNode;
    }

    /**
     * Parses the given Rule into a network of GrammarNodes.
     * 
     * @param rule
     *            the Rule to parse
     * @return a grammar graph
     */
    private GrammarGraph parseRule(RuleComponent rule) throws GrammarException {
        GrammarGraph result;

        if (rule != null) {
            if (LOGGER.isLoggable(Level.FINE)) {
                LOGGER.fine("parseRule: " + rule.toString());
            }
        }

        if (rule instanceof RuleAlternatives) {
            result = parseRuleAlternatives((RuleAlternatives) rule);
        } else if (rule instanceof RuleCount) {
            result = parseRuleCount((RuleCount) rule);
        } else if (rule instanceof RuleReference) {
            result = parseRuleReference((RuleReference) rule);
        } else if (rule instanceof RuleSequence) {
            result = parseRuleSequence((RuleSequence) rule);
        } else if (rule instanceof RuleTag) {
            result = parseRuleTag((RuleTag) rule);
        } else if (rule instanceof RuleToken) {
            result = parseRuleToken((RuleToken) rule);
        } else if (rule instanceof RuleSpecial) {
            result = parseRuleSpecial((RuleSpecial) rule);
        } else if (rule instanceof RuleParse) {
            throw new IllegalArgumentException(
                    "Unsupported Rule type: RuleParse: " + rule.toString());
        } else {
            throw new IllegalArgumentException("Unsupported Rule type: "
                    + rule.toString());
        }
        return result;
    }

    private Rule getExternalGrammar(String grammarReference, String ruleName) {

        SrgsRuleGrammarParser srgsRuleGrammarParser = new SrgsRuleGrammarParser();
        URL url = null;
        InputStream grammarStream = null;

        try {
            if (grammarReference.contains(":")) {
                url = new URL(grammarReference);
            } else {
                url = new URL(baseURL + grammarReference);
            }
            grammarStream = url.openStream();
        } catch (MalformedURLException ex) {
            ex.printStackTrace();
        } catch (IOException ex) {
            ex.printStackTrace();
        }

        Rule rules[] = srgsRuleGrammarParser.load(grammarStream);

        for (Rule r : rules) {
            if (r.getRuleName().compareTo(ruleName) == 0) {
                return r;
            }
        }

        return null;
    }

    /** @todo change this by a proper solution */
    private GrammarGraph parseRuleSpecial(RuleSpecial rule) {
        if (rule == RuleSpecial.NULL) {
            // no connection for null
        } else if (rule == RuleSpecial.VOID) {
            // no connection for void
        } else if (rule == RuleSpecial.GARBAGE) {
            throw new IllegalArgumentException(
                    "This recognizer does not support the GARBAGE rule.");
        }
        GrammarNode node = createGrammarNode(false);
        return new GrammarGraph(node, node);
    }

    /**
     * Parses the given RuleName into a network of GrammarNodes.
     * 
     * @param initialRule
     *            Rule to parse
     * @return a grammar graph
     */
    private GrammarGraph parseRuleReference(RuleReference initialRule)
            throws GrammarException {
        if (LOGGER.isLoggable(Level.FINE)) {
            LOGGER.fine("parseRuleName: " + initialRule.toString());
        }
        GrammarGraph result = (GrammarGraph) ruleStack.contains(initialRule
                .getRuleName());

        if (result != null) { // its a recursive call
            return result;
        } else {
            result = new GrammarGraph();
            ruleStack.push(initialRule.getRuleName(), result);
        }
        RuleReference ruleName = initialRule;

        if (ruleName == null) {
            throw new GrammarException("Can't resolve " + initialRule + " g "
                    + initialRule.getGrammarReference());
        }

        Rule rule;

        if (ruleName.getGrammarReference() == null)
            rule = ruleGrammar.getRule(ruleName.getRuleName());
        else
            rule = getExternalGrammar(ruleName.getGrammarReference(), ruleName
                    .getRuleName());

        if (rule == null) {
            throw new GrammarException("Can't resolve grammar reference: "
                    + ruleName.getGrammarReference() + " with grammar name: "
                    + ruleName.getRuleName());
        }

        if (rule == null) {
            throw new GrammarException("Can't resolve rule: "
                    + ruleName.getRuleName());
        }
        GrammarGraph ruleResult = parseRule(rule.getRuleComponent());
        if (result != ruleResult) {
            result.getStartNode().add(ruleResult.getStartNode(), 0.0f);
            ruleResult.getEndNode().add(result.getEndNode(), 0.0f);
        }

        ruleStack.pop();
        return result;
    }

    /**
     * Parses the given RuleCount into a network of GrammarNodes.
     * 
     * @param ruleCount
     *            the RuleCount object to parse
     * @return a grammar graph
     */
    private GrammarGraph parseRuleCount(RuleCount ruleCount)
            throws GrammarException {
        if (LOGGER.isLoggable(Level.FINE)) {
            LOGGER.fine("parseRuleCount: " + ruleCount);
        }
        GrammarGraph result = new GrammarGraph();
        int minRepeat = ruleCount.getRepeatMin();
        int maxRepeat = ruleCount.getRepeatMax();
        GrammarGraph newNodes = parseRule(ruleCount.getRuleComponent());
        int countNodes = 1;

        GrammarGraph lastNode = newNodes;

        if (minRepeat > 1) {
            GrammarGraph tmpGraph;
            while (countNodes < minRepeat) {
                countNodes++;
                tmpGraph = parseRule(ruleCount.getRuleComponent());
                /** @todo how can i copy a graph */
                lastNode = tmpGraph;
                newNodes.getEndNode().add(tmpGraph.getStartNode(), 0.0f);
                newNodes.endNode = tmpGraph.getEndNode();
                /** @todo review this */
            }
        }

        if (maxRepeat != RuleCount.REPEAT_INDEFINITELY) {
            GrammarGraph tmpGraph;
            ArrayList<GrammarNode> v = new ArrayList();
            lastNode = newNodes;
            while (countNodes < maxRepeat) {
                ++countNodes;
                tmpGraph = parseRule(ruleCount.getRuleComponent());
                /** @todo how can i copy a graph */
                v.add(lastNode.getEndNode());
                newNodes.getEndNode().add(tmpGraph.getStartNode(), 0.0f);
                newNodes.getEndNode().add(tmpGraph.getEndNode(), 0.0f);
                newNodes.endNode = tmpGraph.getEndNode();
                /** @todo review this */
                lastNode = tmpGraph;
            }

            // set this nodes optional
            for (GrammarNode g : v) {
                g.add(newNodes.getEndNode(), 0.0f);
            }
        } else {

        }
        result.getStartNode().add(newNodes.getStartNode(), 0.0f);
        newNodes.getEndNode().add(result.getEndNode(), 0.0f);

        // if this is optional, add a bypass arc

        if (minRepeat == 0) {
            result.getStartNode().add(result.getEndNode(), 0.0f);
        }

        // if this can possibly occur indefinitely add a loopback

        if (maxRepeat == RuleCount.REPEAT_INDEFINITELY) {
            if (lastNode != null) {
                newNodes.getEndNode().add(lastNode.getStartNode(), 0.0f);
            }
        }

        return result;
    }

    /**
     * Parses the given RuleAlternatives into a network of GrammarNodes.
     * 
     * @param ruleAlternatives
     *            the RuleAlternatives to parse
     * @return a grammar graph
     */
    private GrammarGraph parseRuleAlternatives(RuleAlternatives ruleAlternatives)
            throws GrammarException {
        if (LOGGER.isLoggable(Level.FINE)) {
            LOGGER
                    .fine("parseRuleAlternatives: "
                            + ruleAlternatives.toString());
        }
        GrammarGraph result = new GrammarGraph();

        RuleComponent[] rules = ruleAlternatives.getRuleComponents();
        int[] weights = ruleAlternatives.getWeights();
        /** @todo implement it in jsapi2/srgsrulegrammarparser */
        // normalizeWeights(weights);

        // end each alternative, and connect them in parallel
        for (int i = 0; i < rules.length; i++) {
            RuleComponent rule = rules[i];
            float weight = 0.0f;
            if (weights != null) {
                weight = weights[i];
            }
            if (LOGGER.isLoggable(Level.FINE)) {
                LOGGER.fine("Alternative: " + rule.toString());
            }
            GrammarGraph newNodes = parseRule(rule);

            if (newNodes.getStartNode() != null) {
                result.getStartNode().add(newNodes.getStartNode(), weight);
                newNodes.getEndNode().add(result.getEndNode(), 0.0f);
            }
        }

        return result;
    }

    private boolean isRuleDisabled(RuleComponent rule) throws GrammarException {
        /* @todo do it */
        /*
         * RuleName ruleName = ruleGrammar.resolve(new
         * RuleName(rule.toString())); boolean isRuleEnabled = ruleName != null
         * && !ruleGrammar.isEnabled(ruleName.getSimpleRuleName()); return
         * isRuleEnabled;
         */
        return false;
    }

    /**
     * Normalize the weights. The weights should always be zero or greater. We
     * need to convert the weights to a log probability.
     * 
     * @param weights
     *            the weights to normalize
     */
    private void normalizeWeights(float[] weights) {
        if (weights != null) {
            double sum = 0.0;
            for (int i = 0; i < weights.length; i++) {
                if (weights[i] < 0) {
                    throw new IllegalArgumentException("negative weight");
                }
                sum += weights[i];
            }
            for (int i = 0; i < weights.length; i++) {
                if (sum == 0.0f) {
                    weights[i] = LogMath.getLogZero();
                } else {
                    weights[i] = logMath.linearToLog(weights[i] / sum);
                }
            }
        }
    }

    /**
     * Parses the given RuleSequence into a network of GrammarNodes.
     * 
     * @param ruleSequence
     *            the RuleSequence to parse
     * @return the first and last GrammarNodes of the network
     */
    private GrammarGraph parseRuleSequence(RuleSequence ruleSequence)
            throws GrammarException {

        GrammarNode startNode = null;
        GrammarNode endNode = null;
        if (LOGGER.isLoggable(Level.FINE)) {
            LOGGER.fine("parseRuleSequence: " + ruleSequence);
        }

        RuleComponent[] rules = ruleSequence.getRuleComponents();

        GrammarNode lastGrammarNode = null;

        // expand and connect each rule in the sequence serially
        for (int i = 0; i < rules.length; i++) {
            RuleComponent rule = rules[i];
            if (isRuleDisabled(rule))
                continue;

            GrammarGraph newNodes = parseRule(rule);

            // first node
            if (i == 0) {
                startNode = newNodes.getStartNode();
            }

            // last node
            if (i == (rules.length - 1)) {
                endNode = newNodes.getEndNode();
            }

            if (i > 0) {
                lastGrammarNode.add(newNodes.getStartNode(), 0.0f);
            }
            lastGrammarNode = newNodes.getEndNode();
        }

        return new GrammarGraph(startNode, endNode);
    }

    /**
     * Parses the given RuleTag into a network GrammarNodes.
     * 
     * @todo change this by a proper solution.
     * 
     * @param ruleTag
     *            the RuleTag to parse
     * @return the first and last GrammarNodes of the network
     */
    private GrammarGraph parseRuleTag(RuleTag ruleTag) throws GrammarException {
        if (LOGGER.isLoggable(Level.FINE)) {
            LOGGER.fine("parseRuleTag: " + ruleTag);
        }
        GrammarNode node = createGrammarNode(false);
        return new GrammarGraph(node, node);
    }

    /**
     * Creates a GrammarNode with the word in the given RuleToken.
     * 
     * @param ruleToken
     *            the RuleToken that contains the word
     * @return a GrammarNode with the word in the given RuleToken
     */
    private GrammarGraph parseRuleToken(RuleToken ruleToken) {
        String text = ruleToken.getText();
        String[] words = text.split(" ");

        GrammarNode startNode = null;
        GrammarNode endNode = null;

        if (words.length == 1) {
            startNode = createGrammarNode(ruleToken.getText());
            endNode = startNode;
        } else {
            for (int i = 0; i < words.length; ++i) {
                GrammarNode node = createGrammarNode(words[i]);
                if (i == 0) {
                    startNode = node;
                    endNode = node;
                } else {
                    endNode.add(node, 0);
                    endNode = node;
                }
            }
        }

        return new GrammarGraph(startNode, endNode);
    }

    /**
     * Dumps out a grammar exception
     * 
     * @param ge
     *            the grammar exception
     */
    private void dumpGrammarException(GrammarException ge) {
        LOGGER.warning("Grammar exception " + ge);
        GrammarExceptionDetail[] gsd = ge.getDetails();
        if (gsd != null) {
            for (int i = 0; i < gsd.length; i++) {
                LOGGER.warning("Grammar Name: " + gsd[i].getGrammarReference());
                LOGGER.warning("Line number : " + gsd[i].getLineNumber());
                LOGGER.warning("char number : " + gsd[i].getCharNumber());
                LOGGER.warning("Rule name   : " + gsd[i].getRuleName());
                LOGGER.warning("Message     : " + gsd[i].getMessage());
                LOGGER.warning("Text Info     : " + gsd[i].getTextInfo());
                LOGGER.warning("Type        : " + gsd[i].getType());
            }
        }
    }

    /**
     * Commit changes to all loaded grammars and all changes of grammar since
     * the last commitChange
     */
    public void commitChanges() throws IOException {
        try {

            SrgsRuleGrammarParser srgsRuleGrammarParser = new SrgsRuleGrammarParser();

            Rule rules[];

            if (grammarURL != null) {
                InputStream grammarStream = grammarURL.openStream();
                rules = srgsRuleGrammarParser.load(grammarStream);
            } else {
                rules = srgsRuleGrammarParser.load(new InputStreamReader(
                        new ByteArrayInputStream(grammarSTR.getBytes())));
            }

            if (rules != null) {
                ruleGrammar = new BaseRuleGrammar(null, grammarName);
                /** @todo dgmr cuidado com isto */
                ruleGrammar.addRules(rules);
                // ruleGrammar.setAttributes(srgsRuleGrammarParser.getAttributes());
                // /**@todo nao eh necessario ??? */
            }

            ruleGrammar.commitChanges();

            ruleStack = new RuleStack();
            newGrammar();

            firstNode = createGrammarNode("<sil>");
            GrammarNode finalNode = createGrammarNode("<sil>");
            finalNode.setFinalNode(true);

            // go through each rule and create a network of GrammarNodes
            // for each of them

            String[] ruleNames = ruleGrammar.listRuleNames();
            for (int i = 0; i < ruleNames.length; i++) {
                String ruleName = ruleNames[i];
                if (ruleGrammar.getRule(ruleName).getScope() == Rule.PUBLIC) {
                    String fullName = getFullRuleName(ruleName);
                    GrammarGraph publicRuleGraph = new GrammarGraph();
                    ruleStack.push(fullName, publicRuleGraph);
                    RuleComponent rule = ruleGrammar.getRule(ruleName)
                            .getRuleComponent();
                    GrammarGraph graph = parseRule(rule);
                    ruleStack.pop();

                    firstNode.add(publicRuleGraph.getStartNode(), 0.0f);
                    publicRuleGraph.getEndNode().add(finalNode, 0.0f);
                    publicRuleGraph.getStartNode().add(graph.getStartNode(),
                            0.0f);
                    graph.getEndNode().add(publicRuleGraph.getEndNode(), 0.0f);
                }
            }
            postProcessGrammar();
        } catch (GrammarException ge) {
            dumpGrammarException(ge);
            throw new IOException("GrammarException: " + ge);
        } catch (EngineStateException ex) {
        }
    }

    /**
     * Gets the fully resolved rule name
     * 
     * @param ruleName
     *            the partial name
     * @return the fully resovled name
     * @throws GrammarException
     */
    private String getFullRuleName(String ruleName) throws GrammarException {
        /*
         * RuleReference rname = ruleGrammar.resolve(new
         * RuleReference(ruleName)); return rname.getRuleName();
         *//**
         * @todo review this implementation; our implementation of base
         *       engines not implementation resolve method
         */
        return ruleName;
    }

    /**
     * Represents a graph of grammar nodes. A grammar graph has a single
     * starting node and a single ending node
     */
    class GrammarGraph {

        private GrammarNode startNode;
        private GrammarNode endNode;

        /**
         * Creates a grammar graph with the given nodes
         * 
         * @param startNode
         *            the staring node of the graph
         * @param endNode
         *            the ending node of the graph
         */
        GrammarGraph(GrammarNode startNode, GrammarNode endNode) {
            this.startNode = startNode;
            this.endNode = endNode;
        }

        /** Creates a graph with non-word nodes for the start and ending nodes */
        GrammarGraph() {
            startNode = createGrammarNode(false);
            endNode = createGrammarNode(false);
        }

        /**
         * Gets the starting node
         * 
         * @return the starting node for the graph
         */
        GrammarNode getStartNode() {
            return startNode;
        }

        /**
         * Gets the ending node
         * 
         * @return the ending node for the graph
         */
        GrammarNode getEndNode() {
            return endNode;
        }
    }

    /** Manages a stack of grammar graphs that can be accessed by grammar name */
    class RuleStack {

        private List stack;
        private HashMap map;

        /** Creates a name stack */
        public RuleStack() {
            clear();
        }

        /** Pushes the grammar graph on the stack */
        public void push(String name, GrammarGraph g) {
            stack.add(0, name);
            map.put(name, g);
        }

        /** remove the top graph on the stack */
        public void pop() {
            map.remove(stack.remove(0));
        }

        /**
         * Checks to see if the stack contains a graph with the given name
         * 
         * @param name
         *            the graph name
         * @return the grammar graph associated with the name if found,
         *         otherwise null
         */
        public GrammarGraph contains(String name) {
            if (stack.contains(name)) {
                return (GrammarGraph) (GrammarGraph) map.get(name);
            } else {
                return null;
            }
        }

        /** Clears this name stack */
        public void clear() {
            stack = new LinkedList();
            map = new HashMap();
        }
    }
}
