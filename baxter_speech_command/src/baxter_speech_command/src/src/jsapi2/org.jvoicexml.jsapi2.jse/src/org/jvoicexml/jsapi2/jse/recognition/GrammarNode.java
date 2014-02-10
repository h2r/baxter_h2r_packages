package org.jvoicexml.jsapi2.jse.recognition;

import java.util.ArrayList;
import java.util.List;

import javax.speech.recognition.RuleComponent;

/**
 * Represents a node of a graph.
 *
 * @author David José Rodrigues
 */

public class GrammarNode {

    /** Represent an end alternative node. */
    public static final int END_ALTERNATIVE = 1;
    /** Represent an end count node. */
    public static final int END_COUNT = 2;
    /** Represent an end reference node. */
    public static final int END_REFERENCE = 3;
    /** Represent an end sequence node. */
    public static final int END_SEQUENCE = 4;
    /** Represent an start alternative node. */
    public static final int START_ALTERNATIVE = 5;
    /** Represent an start count node. */
    public static final int START_COUNT = 6;
    /** Represent an start reference node. */
    public static final int START_REFERENCE = 7;
    /** Represent an start sequence node. */
    public static final int START_SEQUENCE = 8;
    /** Represent an tag node. */
    public static final int TAG = 9;
    /** Represent an token node. */
    public static final int TOKEN = 10;
    
    /** Represent an token node. */
    public static final int SPECIAL = 11;

    /** <code>true</code> if this node is a final node of the graph. */
    private boolean isFinal;

    /** the arcs to the successors nodes. */
    private List<GrammarArc> arcList = new ArrayList<GrammarArc>();

    /** the type of this node. */
    private int type;

    /** the rule component associated with this node. */
    private RuleComponent ruleComponent;

    /**
     * Creates a grammar node without a rule component associated.
     * @param isFinalNode boolean
     * @param nodeType the node type
     * @param rc RuleComponent the associated rule component
     */
    protected GrammarNode(final boolean isFinalNode, final int nodeType,
                          final RuleComponent rc) {
        this.isFinal = isFinalNode;
        this.type = nodeType;
        this.ruleComponent = rc;
    }

    /**
     * Create a grammar node, without a rule component associated.
     * @param isFinalNode boolean
     * @param nodeType int
     */
    protected GrammarNode(final boolean isFinalNode, final int nodeType) {
        this.isFinal = isFinalNode;
        this.type = nodeType;
        this.ruleComponent = null;
    }

    /**
     * Checks if this node is a final node.
     * @return <code>true</code> if this is a final node.
     */
    public final boolean isFinalNode() {
        return isFinal;
    }

    /**
     * Adds an arc, from this node to the destinationNode.
     * @param destinationNode the destination node
     */
    public final void addArc(final GrammarNode destinationNode) {
        arcList.add(new GrammarArc(destinationNode));
    }

    /**
     * Gets the node type.
     * @return int
     */
    public final int getNodeType() {
        return type;
    }

    /**
     * Gets the arc list.
     * @return List
     */
    public final List<GrammarArc> getArcList() {
        return arcList;
    }

    /**
     * Gets the rule component associated with this node.
     * @return RuleComponent
     */
    public final RuleComponent getRuleComponent() {
        return ruleComponent;
    }

}
