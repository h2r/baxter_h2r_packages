/**
 * Copyright 1998-2003 Sun Microsystems, Inc.
 *
 * See the file "license.terms" for information on usage and
 * redistribution of this file, and for a DISCLAIMER OF ALL
 * WARRANTIES.
 */

package org.jvoicexml.jsapi2.jse.recognition;

import java.util.ArrayList;
import java.util.Stack;
import java.util.StringTokenizer;
import java.util.Vector;
import java.util.logging.Logger;

import javax.speech.EngineStateException;
import javax.speech.recognition.Grammar;
import javax.speech.recognition.GrammarManager;
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


/**
 * Implementation of the parse method(s) on
 * javax.speech.recognition.RuleGrammar.
 *
 * @version 1.5 10/27/99 16:33:49
 */
public class RuleParser {
    /** Logger for this class. */
    private static final Logger LOGGER =
            Logger.getLogger(RuleParser.class.getName());

    /** the grammarManager that contains all the grammars. */
    private GrammarManager grammarManager;

    /** Represents the current position of the input array,
     * during the parse algorithm.
     */
    private int iPos;

    /** This stack helps the construction of the RuleParse in
     * Deepth-search algorithm.
     */
    private Stack<Object> grammarElements;

    /**
     * Creates a Rule Parser.
     * @param gm GrammarManager
     * @param pos int
     */
    public RuleParser(final GrammarManager gm, final int pos) {
        this.grammarManager = gm;
        iPos = pos;
        grammarElements = new Stack<Object>();
    }

    /**
     * Parse a text string against a particular rule from a particluar grammar
     * returning a RuleParse data structure is successful and null otherwise.
     * @param text text to search
     * @param grammarManager the grammar manager
     * @param grammarReference the grammar reference
     * @param ruleName the start rule name
     * @return RuleParse
     */
    public static RuleParse parse(final String text,
                                  final GrammarManager grammarManager,
                                  final String grammarReference,
                                  final String ruleName) {
        String[] inputTokens = tokenize(text);
        return parse(inputTokens, grammarManager, grammarReference, ruleName);
    }

    /**
     * Parse a set of tokens against a particular rule from a particluar grammar
     * returning a RuleParse data structure is successful and null otherwise.
     * @param inputTokens String[]
     * @param grammarManager GrammarManager
     * @param grammarReference String
     * @param ruleName String
     * @return RuleParse the start rule name
     */
    public static RuleParse parse(final String[] inputTokens,
                                  final GrammarManager grammarManager,
                                  final String grammarReference,
                                  final String ruleName) {
        RuleParse[] rpa = mparse(inputTokens, grammarManager, grammarReference,
                                 ruleName);
        if (rpa == null) {
            return null;
        } else {
            return rpa[0];
        }
    }

    /**
     * Parse a text against a particular rule from a particluar grammar
     * returning a RuleParse data structure is successful and null otherwise.
     * @param text String
     * @param grammarManager GrammarManager
     * @param grammarReference String
     * @param ruleName String
     * @return RuleParse[]
     */
    public static RuleParse[] mparse(final String text,
                                     final GrammarManager grammarManager,
                                     final String grammarReference,
                                     final String ruleName) {
        String[] inputTokens = tokenize(text);
        return mparse(inputTokens, grammarManager, grammarReference, ruleName);
    }

    /**
     * Parse a set of tokens against a particular rule from a particluar grammar
     * returning a RuleParse data structure is successful and null otherwise.
     * @param inputTokens String[]
     * @param grammarManager GrammarManager
     * @param grammarReference String
     * @param ruleName String
     * @return RuleParse[]
     */
    public static RuleParse[] mparse(final String[] inputTokens,
                                     final GrammarManager grammarManager,
                                     final String grammarReference,
                                     final String ruleName) {
        final RuleParser rp = new RuleParser(grammarManager, 0);
        String[] rNames;
        RuleComponent startRule = null;
        final Grammar gram = grammarManager.getGrammar(grammarReference);
        final RuleGrammar grammar;
        if (gram instanceof RuleGrammar) {
            grammar = (RuleGrammar) gram;
        } else {
            return null;
        }
        if (ruleName != null) {
            rNames = new String[1];
            rNames[0] = ruleName;
        } else {
            rNames = grammar.listRuleNames();
        }
        Vector parsed = new Vector();
        for (int j = 0; j < rNames.length; j++) {
            if ((ruleName == null) && !(grammar.isActivatable(rNames[j]))) {
                continue;
            }
            final Rule currentRule = grammar.getRule(rNames[j]);
            startRule = currentRule.getRuleComponent();
            if (startRule == null) {
                LOGGER.severe("BAD RULENAME " + rNames[j]);
                continue;
            }
            final GrammarGraph grammarGraph = rp.buildGrammarGraph(grammar,
                    rNames[j]);
            final GrammarNode node = grammarGraph.getStartNode();
            if (rp.parse(node, inputTokens)) {
                final Object element = rp.grammarElements.pop();
                parsed.add(element);
            }
        }
        if (parsed.size() == 0) {
            //No parse is available
            return null;
        }
        final RuleParse[] rpa = new RuleParse[parsed.size()];
        parsed.copyInto(rpa);
        return rpa;
    }

    /**
     * this method starts the recursively of the parse.
     * @param currentNode the start node
     * @param input the text
     * @return RuleComponent
     */
    public final RuleComponent parse(final GrammarNode currentNode,
                               final String input) {
        String[] in = tokenize(input);
        iPos = 0;
        grammarElements = new Stack<Object>();
        if (parse(currentNode, in) && !grammarElements.empty()) {
            return (RuleComponent) grammarElements.pop();
        } else {
            return null;
        }
    }

    /**
     * Creates a grammar graph, from a rule grammar and a start rule name.
     * @param rg the rule grammar
     * @param startRuleName the start rule name
     * @return GrammarGraph the graph that represents this grammar
     */
    public final GrammarGraph buildGrammarGraph(final RuleGrammar rg,
                                          final String startRuleName) {
        RuleComponent startRuleComponent = rg.getRule(startRuleName).
                                           getRuleComponent();
        RuleReference startRule = new RuleReference(startRuleName);

        GrammarNode startNode = new GrammarNode(false,
                                                GrammarNode.START_REFERENCE,
                                                startRule);
        GrammarNode endNode = new GrammarNode(true, GrammarNode.END_REFERENCE);
        GrammarGraph newNodes = buildGrammarGraph(rg, startRuleComponent);

        startNode.addArc(newNodes.getStartNode());
        newNodes.getEndNode().addArc(endNode);

        return new GrammarGraph(startNode, endNode);
    }

    /**
     * this method processes recursively an rule component.
     * It returns the an GrammarGraph
     * @param rg the rule grammar
     * @param r the rule component
     * @return GrammarGraph
     */
    private GrammarGraph buildGrammarGraph(final RuleGrammar rg,
                                                 final RuleComponent r) {
        
//        if(LOGGER.isDebugEnabled()){
//            LOGGER.debug("RuleComponent : "+ r.getClass());
////            if (r instanceof RuleReference) {
////                LOGGER.debug("RuleComponent : "+ ((RuleReference)r).getRuleName());
////            }
//        }
        
        if (r instanceof RuleReference) {
            return buildGrammarGraph(rg, (RuleReference) r);
        } else if (r instanceof RuleToken) {
            return buildGrammarGraph(rg, (RuleToken) r);
        } else if (r instanceof RuleAlternatives) {
            return buildGrammarGraph(rg, (RuleAlternatives) r);
        } else if (r instanceof RuleSequence) {
            return buildGrammarGraph(rg, (RuleSequence) r);
        } else if (r instanceof RuleTag) {
            return buildGrammarGraph(rg, (RuleTag) r);
        } else if (r instanceof RuleCount) {
            return buildGrammarGraph(rg, (RuleCount) r);
        } 
//          else if (r instanceof RuleSpecial) {                                  //____________________________________
//            return buildGrammarGraph(rg, (RuleSpecial) r);
//        } 
        
        else {
            return null;
        }
    }
    
    /**
     * Creates an sub-graph that represents a ruleref Special.
     * @param rg the rule grammar
     * @param r the rule tag
     * @return GrammarGraph
     */
    private GrammarGraph buildGrammarGraph(final RuleGrammar rg,
                                           final RuleSpecial r) {
        GrammarNode startNode = new GrammarNode(false, GrammarNode.SPECIAL, r);
        return new GrammarGraph(startNode, startNode);
    }

    /**
    * Creates an sub-graph that represents a rule reference.
    * @param rg the rule grammar
    * @param r the rule reference
    * @return GrammarGraph
    */
   private GrammarGraph buildGrammarGraph(final RuleGrammar rg,
                                          final RuleReference r) {
       RuleGrammar currentRuleGrammar = rg;
       GrammarNode startNode = new GrammarNode(false,
                                               GrammarNode.START_REFERENCE, r);
       GrammarNode endNode = new GrammarNode(false, GrammarNode.END_REFERENCE);

       String simpleName = r.getRuleName();
       RuleComponent ruleref;
       if (currentRuleGrammar.getRule(simpleName) == null) {
           ruleref = null;
       } else {
           ruleref = currentRuleGrammar.getRule(simpleName).getRuleComponent();
       }
       if (ruleref == null) {
           String gname = r.getGrammarReference();
           if ((gname != null) && (gname.length() > 0)) {
               RuleGrammar rg1 = null;
               try {
                   rg1 = (RuleGrammar) grammarManager.getGrammar(gname);
               } catch (EngineStateException ex) {
                   ex.printStackTrace();
               }
               if (rg1 != null) {
                   ruleref = rg1.getRule(simpleName).getRuleComponent();
                   currentRuleGrammar = rg1;
               } else {
                   LOGGER.severe("ERROR: UNKNOWN GRAMMAR " + gname);
               }
           }
           if (ruleref == null) {
               LOGGER.severe("ERROR: UNKNOWN RULE NAME " + r.getRuleName()
                              + " " + r);
               return null;
           }
       }

       GrammarGraph rc = buildGrammarGraph(currentRuleGrammar, ruleref);
       if (rc == null) {
           return null;
       }
       startNode.addArc(rc.getStartNode());
       rc.getEndNode().addArc(endNode);

       return new GrammarGraph(startNode, endNode);

   }

    /**
    * Creates an sub-graph that represents a rule token.
    * @param rg the rule grammar
    * @param r the rule token
    * @return GrammarGraph
    */
    private GrammarGraph buildGrammarGraph(final RuleGrammar rg,
                                           final RuleToken r) {
        GrammarNode startNode = new GrammarNode(false, GrammarNode.TOKEN, r);
        return new GrammarGraph(startNode, startNode);
    }

    /**
    * Creates an sub-graph that represents a rule alternative.
    * @param rg the rule grammar
    * @param r the rule alternative
    * @return GrammarGraph
    */
    private GrammarGraph buildGrammarGraph(final RuleGrammar rg,
                                           final RuleAlternatives r) {
        GrammarNode startNode = new GrammarNode(false,
                                                GrammarNode.START_ALTERNATIVE,
                                                r);
        GrammarNode endNode = new GrammarNode(false,
                                              GrammarNode.END_ALTERNATIVE);

        RuleComponent[] rules = r.getRuleComponents();
        int[] weights = r.getWeights();
        // @todo implement it in jsapi2/srgsrulegrammarparser
        //normalizeWeights(weights);

        // expand each alternative, and connect them in parallel
        for (int i = 0; i < rules.length; i++) {
            RuleComponent rule = rules[i];
            float weight = 0.0f;
            if (weights != null) {
                weight = weights[i];
            }
            GrammarGraph newNodes = buildGrammarGraph(rg, rule);

            if (newNodes.getStartNode() != null) {
                startNode.addArc(newNodes.getStartNode()); //@todo ??weight?
                newNodes.getEndNode().addArc(endNode);
            }
        }

        return new GrammarGraph(startNode, endNode);

    }

    /**
    * Creates an sub-graph that represents a rule sequence.
    * @param rg the rule grammar
    * @param r the rule sequence
    * @return GrammarGraph
    */
    private GrammarGraph buildGrammarGraph(final RuleGrammar rg,
                                           final RuleSequence r) {
        GrammarNode startNode = new GrammarNode(false,
                                                GrammarNode.START_SEQUENCE, r);
        GrammarNode endNode = new GrammarNode(false, GrammarNode.END_SEQUENCE);

        RuleComponent[] rules = r.getRuleComponents();

        GrammarNode lastGrammarNode = null;

        // expand and connect each rule in the sequence serially
        for (int i = 0; i < rules.length; i++) {
            RuleComponent rule = rules[i];
            GrammarGraph newNodes = buildGrammarGraph(rg, rule);

            // first node
            if (i == 0) {
                startNode.addArc(newNodes.getStartNode());
            }

            // last node
            if (i == (rules.length - 1)) {
                newNodes.getEndNode().addArc(endNode);
            }

            if (i > 0) {
                lastGrammarNode.addArc(newNodes.getStartNode());
            }
            lastGrammarNode = newNodes.getEndNode();
        }

        return new GrammarGraph(startNode, endNode);
    }

    /**
    * Creates an sub-graph that represents a rule tag.
    * @param rg the rule grammar
    * @param r the rule tag
    * @return GrammarGraph
    */
   private GrammarGraph buildGrammarGraph(final RuleGrammar rg,
                                          final RuleTag r) {
       GrammarNode startNode = new GrammarNode(false, GrammarNode.TAG, r);
       return new GrammarGraph(startNode, startNode);
   }

    /**
    * Creates an sub-graph that represents a rule count.
    * @param rg the rule grammar
    * @param r the rule count
    * @return GrammarGraph
    */
   private GrammarGraph buildGrammarGraph(final RuleGrammar rg,
                                          final RuleCount r) {
       GrammarNode startNode = new GrammarNode(false, GrammarNode.START_COUNT,
                                               r);
       GrammarNode endNode = new GrammarNode(false, GrammarNode.END_COUNT);

       int minRepeat = r.getRepeatMin();
       int maxRepeat = r.getRepeatMax();
       GrammarGraph newNodes = buildGrammarGraph(rg, r.getRuleComponent());
       int countNodes = 1;

       GrammarGraph lastNode = newNodes;

       if (minRepeat > 1) {
           GrammarGraph tmpGraph;
           while (countNodes < minRepeat) {
               countNodes++;
               /** @todo how can i copy a graph */
               tmpGraph = buildGrammarGraph(rg, r.getRuleComponent());
               lastNode = tmpGraph;
               newNodes.getEndNode().addArc(tmpGraph.getStartNode());
               /** @todo review this */
               newNodes.setEndNode(tmpGraph.getEndNode());
           }
       }

       if (maxRepeat != RuleCount.REPEAT_INDEFINITELY) {
           GrammarGraph tmpGraph;
           ArrayList<GrammarNode> v = new ArrayList();
           lastNode = newNodes;
           while (countNodes < maxRepeat) {
               ++countNodes;
               /** @todo how can i copy a graph */
               tmpGraph = buildGrammarGraph(rg, r.getRuleComponent());
               v.add(lastNode.getEndNode());
               newNodes.getEndNode().addArc(tmpGraph.getStartNode());
               /** @todo review this */
               newNodes.setEndNode(tmpGraph.getEndNode());
               lastNode = tmpGraph;
           }

           //set this nodes optional
           for (GrammarNode g : v) {
               g.addArc(endNode);
           }
       }

       startNode.addArc(newNodes.getStartNode());
       newNodes.getEndNode().addArc(endNode);

       // if this is optional, add a bypass arc
       if (minRepeat == 0) {
           startNode.addArc(endNode);
       }

       // if this can possibly occur indefinitely add a loopback
       if (maxRepeat == RuleCount.REPEAT_INDEFINITELY) {
           if (lastNode != null) {
               newNodes.getEndNode().addArc(lastNode.getStartNode());
           }
       }
       return new GrammarGraph(startNode, endNode);
   }

   /** this method parses a token.
     * @param currentNode the current node of this grammar
     * @param input the set of tokens
     * @return <code>true</code> if this nodes accepts the current (iPos) input
     */
    public final boolean parseToken(final GrammarNode currentNode,
                                    final String[] input) {
        if (iPos >= input.length) {
            return false;
        }
        // @todo what about case sensitivity ??????
        String tText = ((RuleToken) currentNode.getRuleComponent()).getText().
                       toLowerCase();
        if (tText.equals(input[iPos]) || (input[iPos].equals("%"))
            || (input[iPos].equals("*"))) {
            iPos++;
            if (parse(currentNode.getArcList().get(0).getGrammarNode(),
                      input)) {
                grammarElements.push(new RuleToken(tText));
                return true;
            } else {
                return false;
            }
        } else {
            if (tText.indexOf(' ') < 0) {
                return false;
            }
            if (!tText.startsWith(input[iPos])) {
                return false;
            }
            String[] ta = tokenize(tText);
            int j = 0;
            StringBuffer strBuffer = new StringBuffer("");
            // this while is necessary because an token
            // can contain more than a single word
            while (true) {
                if (j >= ta.length) {
                    break;
                }
                if (iPos >= input.length) {
                    return false;
                }
                if (!ta[j].equals(input[iPos])) {
                    return false;
                }
                if (j > 0) {
                    strBuffer.append(" ");
                }
                strBuffer.append(ta[j]);
                iPos++;
                j++;
            }
            if (parse(currentNode.getArcList().get(0).getGrammarNode(),
                      input)) {
                grammarElements.push(new RuleToken(strBuffer.toString()));
                return true;
            } else {
                return false;
            }
        }
    }

    /**
     * This method constructs an RuleSequence for the rule parser.
     */
    private void posParseStartSequence() {
        Vector<RuleComponent> arSeq = new Vector<RuleComponent>();
        while (true) {
            if (grammarElements.empty()) {
                break;
            }
            Object topElement = grammarElements.pop();
            if (topElement instanceof GrammarNode
                && ((GrammarNode) topElement).getNodeType()
                == GrammarNode.END_SEQUENCE) {
                RuleComponent[] rc = new RuleComponent[arSeq.size()];
                arSeq.copyInto(rc);
                grammarElements.push(new RuleSequence(rc));
                break;
            } else if (topElement instanceof RuleComponent) {
                arSeq.add((RuleComponent) topElement);
            }
        }
    }

    /**
     * This method constructs an RuleCount for the rule parser.
     * @param currentNode the current node
     */
    private void posParseStartCount(final GrammarNode currentNode) {
        Vector ruleComponents = new Vector();
        int count = 0;
        while (true) {
            if (grammarElements.empty()) {
                break;
            }
            Object topElement = grammarElements.pop();
            if (topElement instanceof GrammarNode
                && ((GrammarNode) topElement).getNodeType()
                == GrammarNode.END_COUNT) {
                RuleComponent[] copy = new RuleComponent[ruleComponents.size()];
                ruleComponents.copyInto(copy);

                int repeatProb = ((RuleCount) currentNode.getRuleComponent()).
                                 getRepeatProbability();
                if (repeatProb != RuleCount.MAX_PROBABILITY) {
                    grammarElements.push(new RuleCount(new RuleSequence(copy),
                            count, count, repeatProb));
                } else {
                    grammarElements.push(new RuleCount(new RuleSequence(copy),
                            count, count));
                }
                break;
            } else if (topElement instanceof RuleComponent) {
                ruleComponents.add(topElement);
                count++;
            }
        }
    }

    /**
     * This method constructs an RuleReference for the rule parser.
     * @param currentNode the current node
     */
    private void posParseStartReference(final GrammarNode currentNode) {
        RuleComponent arReference = null;
        while (true) {
            if (grammarElements.empty()) {
                break;
            }
            Object topElement = grammarElements.pop();
            if (topElement instanceof GrammarNode
                && ((GrammarNode) topElement).getNodeType()
                == GrammarNode.END_REFERENCE) {
                String ruleName = ((RuleReference) currentNode.
                                   getRuleComponent()).getRuleName();
                grammarElements.push(new RuleParse(new RuleReference(ruleName),
                        arReference));
                break;
            } else if (topElement instanceof RuleComponent) {
                if (arReference == null) {
                    arReference = (RuleComponent) topElement;
                }
            }
        }
    }

    /**
     * This method constructs an RuleAlternative for the rule parser.
     */
    private void posParseStartAlternative() {
        Vector<RuleComponent> arAlternative = new Vector<RuleComponent>();
        while (true) {
            if (grammarElements.empty()) {
                break;
            }
            Object topElement = grammarElements.pop();
            if (topElement instanceof GrammarNode
                && ((GrammarNode) topElement).getNodeType()
                == GrammarNode.END_ALTERNATIVE) {
                RuleComponent[] rc = new RuleComponent[arAlternative.size()];
                arAlternative.copyInto(rc);
                grammarElements.add(new RuleAlternatives(rc));
                break;
            } else if (topElement instanceof RuleComponent) {
                arAlternative.add((RuleComponent) topElement);
            }
        }
    }

    /** this method parses recursively an grammar graph, to check if
     * a set of tokens belongs at this grammar.
     * @param currentNode the current node of this grammar
     * @param input the set of tokens
     * @return <code>true</code> if starting in the current node exists an
     * way in the graph to ends in a final node.
     */
    private boolean parse(final GrammarNode currentNode,
                                final String[] input) {
        final int currentIPos = iPos;

        if (currentNode.isFinalNode()) {
            if (iPos == input.length) {
                grammarElements.push(currentNode);
                return true;
            } else {
                return false;
            }
        }

        if (currentNode.getNodeType() == GrammarNode.TOKEN) {
            return parseToken(currentNode, input);
        } else {
            for (GrammarArc arc : currentNode.getArcList()) {
                if (parse(arc.getGrammarNode(), input)) {
                    switch (currentNode.getNodeType()) {
                    case GrammarNode.END_ALTERNATIVE:
                    case GrammarNode.END_COUNT:
                    case GrammarNode.END_REFERENCE:
                    case GrammarNode.END_SEQUENCE:
                        grammarElements.push(currentNode);
                        break;
                    case GrammarNode.START_SEQUENCE:
                        posParseStartSequence();
                        break;
                    case GrammarNode.START_COUNT:
                        posParseStartCount(currentNode);
                        break;
                    case GrammarNode.START_REFERENCE:
                        posParseStartReference(currentNode);
                        break;
                    case GrammarNode.START_ALTERNATIVE:
                        posParseStartAlternative();
                        break;
                    default:
                        break;
                    }
                    return true;
                }
            }
            iPos = currentIPos;
            return false;
        }
    }


    /**
     * tokenize a string.
     * @param text the text to tokenize
     * @return an array of tokens
     **/
    static String[] tokenize(final String text) {
        StringTokenizer st = new StringTokenizer(text);
        int size = st.countTokens();
        String[] res = new String[size];
        int i = 0;
        while (st.hasMoreTokens()) {
            res[i++] = st.nextToken().toLowerCase();
        }
        return res;
    }

}


