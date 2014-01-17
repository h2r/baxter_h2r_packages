package javax.speech.recognition;

import java.io.IOException;
import java.io.InputStream;
import java.io.Reader;
import java.util.Vector;

import javax.speech.EngineException;
import javax.speech.EngineStateException;
import javax.speech.SpeechLocale;

//Comp. 2.0.6

public interface GrammarManager {
    void addGrammarListener(GrammarListener listener);

    void removeGrammarListener(GrammarListener listener);

    RuleGrammar createRuleGrammar(String grammarReference, String rootName)
            throws IllegalArgumentException, EngineStateException,
            EngineException;

    RuleGrammar createRuleGrammar(String grammarReference, String rootName,
            SpeechLocale locale) throws IllegalArgumentException,
            EngineStateException, EngineException;

    void deleteGrammar(Grammar grammar) throws IllegalArgumentException,
            EngineStateException;

    Grammar getGrammar(String grammarReference) throws EngineStateException;

    int getGrammarMask();

    void setGrammarMask(int mask);

    Grammar[] listGrammars() throws EngineStateException;

    Grammar loadGrammar(String grammarReference, String mediaType)
            throws GrammarException, IllegalArgumentException, IOException,
            EngineStateException, EngineException;

    Grammar loadGrammar(String grammarReference, String mediaType,
            boolean loadReferences, boolean reloadReferences,
            Vector loadedGrammars) throws GrammarException,
            IllegalArgumentException, IOException, EngineStateException,
            EngineException;

    Grammar loadGrammar(String grammarReference, String mediaType,
            InputStream byteStream, String encoding) throws GrammarException,
            IllegalArgumentException, IOException, EngineStateException,
            EngineException;

    Grammar loadGrammar(String grammarReference, String mediaType,
            Reader charStream) throws GrammarException,
            IllegalArgumentException, IOException, EngineStateException,
            EngineException;

    Grammar loadGrammar(String grammarReference, String mediaType,
            String grammarText) throws GrammarException,
            IllegalArgumentException, IOException, EngineStateException,
            EngineException;
}
