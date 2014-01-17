package org.jvoicexml.jsapi2.jse.recognition;

/**
 * Represents an arc of a graph.
 *
 * @author David José Rodrigues
 */
public class GrammarArc {
    /** the destination node. */
    private GrammarNode grammarNode;

    /**
     * Constructs a new grammar arc.
     * @param node the destination node
     */
    public GrammarArc(final GrammarNode node) {
        this.grammarNode = node;
    }

    /**
     * returns the destination grammar node.
     * @return GrammarNode
     */
    public final GrammarNode getGrammarNode() {
        return grammarNode;
    }

}
