// Generated from C:\Users\Delta2-PC\Desktop\Uni\SimLang\out\si\_parser\SIAntlr.g4 by ANTLR 4.5.1

package si._parser;

import org.antlr.v4.runtime.Lexer;
import org.antlr.v4.runtime.CharStream;
import org.antlr.v4.runtime.Token;
import org.antlr.v4.runtime.TokenStream;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.atn.*;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.misc.*;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast"})
public class SIAntlrLexer extends Lexer {
	static { RuntimeMetaData.checkVersion("4.5.1", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		TFloatPointUnitNumber=1, THexUnitNumber=2, TUnitNumber=3, TUnitInf=4, 
		TComplexNumber=5, Name=6, WS=7, SL_COMMENT=8, ML_COMMENT=9;
	public static String[] modeNames = {
		"DEFAULT_MODE"
	};

	public static final String[] ruleNames = {
		"TFloatPointUnitNumber", "Significant", "THexUnitNumber", "TUnitNumber", 
		"TUnitInf", "TComplexNumber", "RealNumber", "PosNumber", "PosInt", "UngroupedPosInt", 
		"GroupedPosInt", "PosIntGroup", "Unit", "ImperialUnit", "OfficallyAcceptedUnit", 
		"SIUnit", "UnitPrefix", "SiUnitBaseDimension", "SiUnitDimensionless", 
		"Space", "NamePart", "Name", "NEWLINE", "WS", "SL_COMMENT", "ML_COMMENT"
	};

	private static final String[] _LITERAL_NAMES = {
	};
	private static final String[] _SYMBOLIC_NAMES = {
		null, "TFloatPointUnitNumber", "THexUnitNumber", "TUnitNumber", "TUnitInf", 
		"TComplexNumber", "Name", "WS", "SL_COMMENT", "ML_COMMENT"
	};
	public static final Vocabulary VOCABULARY = new VocabularyImpl(_LITERAL_NAMES, _SYMBOLIC_NAMES);

	/**
	 * @deprecated Use {@link #VOCABULARY} instead.
	 */
	@Deprecated
	public static final String[] tokenNames;
	static {
		tokenNames = new String[_SYMBOLIC_NAMES.length];
		for (int i = 0; i < tokenNames.length; i++) {
			tokenNames[i] = VOCABULARY.getLiteralName(i);
			if (tokenNames[i] == null) {
				tokenNames[i] = VOCABULARY.getSymbolicName(i);
			}

			if (tokenNames[i] == null) {
				tokenNames[i] = "<INVALID>";
			}
		}
	}

	@Override
	@Deprecated
	public String[] getTokenNames() {
		return tokenNames;
	}

	@Override

	public Vocabulary getVocabulary() {
		return VOCABULARY;
	}



	// Add additional Java Code to lexer

	private de.monticore.antlr4.MCParser _monticore_parser;
	protected de.monticore.antlr4.MCParser getCompiler() {
	   return _monticore_parser;
	}
	public void setMCParser(de.monticore.antlr4.MCParser in) {
	  this._monticore_parser = in;
	}


	public SIAntlrLexer(CharStream input) {
		super(input);
		_interp = new LexerATNSimulator(this,_ATN,_decisionToDFA,_sharedContextCache);
	}

	@Override
	public String getGrammarFileName() { return "SIAntlr.g4"; }

	@Override
	public String[] getRuleNames() { return ruleNames; }

	@Override
	public String getSerializedATN() { return _serializedATN; }

	@Override
	public String[] getModeNames() { return modeNames; }

	@Override
	public ATN getATN() { return _ATN; }

	@Override
	public void action(RuleContext _localctx, int ruleIndex, int actionIndex) {
		switch (ruleIndex) {
		case 23:
			WS_action((RuleContext)_localctx, actionIndex);
			break;
		case 24:
			SL_COMMENT_action((RuleContext)_localctx, actionIndex);
			break;
		case 25:
			ML_COMMENT_action((RuleContext)_localctx, actionIndex);
			break;
		}
	}
	private void WS_action(RuleContext _localctx, int actionIndex) {
		switch (actionIndex) {
		case 0:
			_channel=HIDDEN;


			break;
		}
	}
	private void SL_COMMENT_action(RuleContext _localctx, int actionIndex) {
		switch (actionIndex) {
		case 1:
			_channel=HIDDEN;
			if (getCompiler()!=null) {
			   de.monticore.ast.Comment _comment = new de.monticore.ast.Comment(getText());
			  _comment.set_SourcePositionStart(new de.se_rwth.commons.SourcePosition(getLine(), getCharPositionInLine()));
			  _comment.set_SourcePositionEnd(getCompiler().computeEndPosition(getToken()));
			  getCompiler().addComment(_comment);
			}


			break;
		}
	}
	private void ML_COMMENT_action(RuleContext _localctx, int actionIndex) {
		switch (actionIndex) {
		case 2:
			_channel=HIDDEN;
			if (getCompiler()!=null) {
			   de.monticore.ast.Comment _comment = new de.monticore.ast.Comment(getText());
			  _comment.set_SourcePositionStart(new de.se_rwth.commons.SourcePosition(getLine(), getCharPositionInLine()));
			  _comment.set_SourcePositionEnd(getCompiler().computeEndPosition(getToken()));
			  getCompiler().addComment(_comment);
			}


			break;
		}
	}

	public static final String _serializedATN =
		"\3\u0430\ud6d1\u8206\uad2d\u4417\uaef1\u8d80\uaadd\2\13\u01b3\b\1\4\2"+
		"\t\2\4\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4"+
		"\13\t\13\4\f\t\f\4\r\t\r\4\16\t\16\4\17\t\17\4\20\t\20\4\21\t\21\4\22"+
		"\t\22\4\23\t\23\4\24\t\24\4\25\t\25\4\26\t\26\4\27\t\27\4\30\t\30\4\31"+
		"\t\31\4\32\t\32\4\33\t\33\3\2\3\2\3\2\3\2\6\2<\n\2\r\2\16\2=\3\3\3\3\3"+
		"\3\6\3C\n\3\r\3\16\3D\3\4\3\4\3\4\3\4\6\4K\n\4\r\4\16\4L\3\5\3\5\7\5Q"+
		"\n\5\f\5\16\5T\13\5\3\5\3\5\7\5X\n\5\f\5\16\5[\13\5\3\5\3\5\7\5_\n\5\f"+
		"\5\16\5b\13\5\5\5d\n\5\3\6\5\6g\n\6\3\6\3\6\3\6\6\6l\n\6\r\6\16\6m\3\6"+
		"\3\6\5\6r\n\6\3\7\3\7\7\7v\n\7\f\7\16\7y\13\7\3\7\3\7\7\7}\n\7\f\7\16"+
		"\7\u0080\13\7\3\7\3\7\3\7\5\7\u0085\n\7\3\b\5\b\u0088\n\b\3\b\3\b\3\t"+
		"\3\t\7\t\u008e\n\t\f\t\16\t\u0091\13\t\3\t\3\t\7\t\u0095\n\t\f\t\16\t"+
		"\u0098\13\t\3\t\3\t\3\t\3\t\3\t\6\t\u009f\n\t\r\t\16\t\u00a0\5\t\u00a3"+
		"\n\t\3\t\3\t\3\t\6\t\u00a8\n\t\r\t\16\t\u00a9\5\t\u00ac\n\t\5\t\u00ae"+
		"\n\t\3\n\3\n\5\n\u00b2\n\n\3\13\3\13\7\13\u00b6\n\13\f\13\16\13\u00b9"+
		"\13\13\3\f\3\f\5\f\u00bd\n\f\3\f\5\f\u00c0\n\f\3\f\6\f\u00c3\n\f\r\f\16"+
		"\f\u00c4\3\r\3\r\3\r\3\r\3\r\3\16\3\16\3\16\5\16\u00cf\n\16\3\17\3\17"+
		"\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17"+
		"\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17"+
		"\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17"+
		"\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17\3\17"+
		"\3\17\3\17\5\17\u010d\n\17\3\20\3\20\3\20\3\20\3\20\3\20\3\20\3\20\3\20"+
		"\3\20\3\20\3\20\3\20\3\20\3\20\3\20\3\20\3\20\3\20\5\20\u0122\n\20\3\21"+
		"\5\21\u0125\n\21\3\21\3\21\3\21\5\21\u012a\n\21\3\21\5\21\u012d\n\21\3"+
		"\21\3\21\5\21\u0131\n\21\3\21\3\21\3\21\3\21\5\21\u0137\n\21\3\21\5\21"+
		"\u013a\n\21\7\21\u013c\n\21\f\21\16\21\u013f\13\21\3\21\5\21\u0142\n\21"+
		"\3\22\3\22\3\22\3\22\5\22\u0148\n\22\3\23\3\23\3\23\3\23\3\23\3\23\3\23"+
		"\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23"+
		"\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23\3\23\5\23"+
		"\u016c\n\23\3\24\3\24\3\24\3\24\3\24\5\24\u0173\n\24\3\25\3\25\3\26\3"+
		"\26\3\27\3\27\3\27\3\27\3\27\3\27\3\27\6\27\u0180\n\27\r\27\16\27\u0181"+
		"\5\27\u0184\n\27\3\30\3\30\3\30\5\30\u0189\n\30\3\31\3\31\3\31\3\31\5"+
		"\31\u018f\n\31\3\31\3\31\3\32\3\32\3\32\3\32\7\32\u0197\n\32\f\32\16\32"+
		"\u019a\13\32\3\32\3\32\3\32\5\32\u019f\n\32\5\32\u01a1\n\32\3\32\3\32"+
		"\3\33\3\33\3\33\3\33\3\33\7\33\u01aa\n\33\f\33\16\33\u01ad\13\33\3\33"+
		"\3\33\3\33\3\33\3\33\2\2\34\3\3\5\2\7\4\t\5\13\6\r\7\17\2\21\2\23\2\25"+
		"\2\27\2\31\2\33\2\35\2\37\2!\2#\2%\2\'\2)\2+\2-\b/\2\61\t\63\n\65\13\3"+
		"\2\24\4\2GGgg\4\2--//\5\2\62;CHch\4\2,,\61\61\4\2ffjj\5\2NNnnvv\n\2GG"+
		"IJMMOORRVV[\\mm\b\2ccefhhoprr{|\7\2CCMMiioouu\6\2EEHHLLXY\4\2JJVV\4\2"+
		"\13\13\"\"\7\2&&\62;C\\aac|\7\2&&C\\aacpr|\6\2&&C\\aac|\b\2&&\62;C\\a"+
		"acpr|\4\2\f\f\17\17\5\2\f\f\17\17,,\u0207\2\3\3\2\2\2\2\7\3\2\2\2\2\t"+
		"\3\2\2\2\2\13\3\2\2\2\2\r\3\2\2\2\2-\3\2\2\2\2\61\3\2\2\2\2\63\3\2\2\2"+
		"\2\65\3\2\2\2\3\67\3\2\2\2\5?\3\2\2\2\7F\3\2\2\2\tN\3\2\2\2\13f\3\2\2"+
		"\2\rs\3\2\2\2\17\u0087\3\2\2\2\21\u00ad\3\2\2\2\23\u00b1\3\2\2\2\25\u00b3"+
		"\3\2\2\2\27\u00ba\3\2\2\2\31\u00c6\3\2\2\2\33\u00ce\3\2\2\2\35\u010c\3"+
		"\2\2\2\37\u0121\3\2\2\2!\u0141\3\2\2\2#\u0147\3\2\2\2%\u016b\3\2\2\2\'"+
		"\u0172\3\2\2\2)\u0174\3\2\2\2+\u0176\3\2\2\2-\u0183\3\2\2\2/\u0188\3\2"+
		"\2\2\61\u018e\3\2\2\2\63\u0192\3\2\2\2\65\u01a4\3\2\2\2\678\5\5\3\289"+
		"\t\2\2\29;\t\3\2\2:<\4\62;\2;:\3\2\2\2<=\3\2\2\2=;\3\2\2\2=>\3\2\2\2>"+
		"\4\3\2\2\2?@\4\62;\2@B\7\60\2\2AC\4\62;\2BA\3\2\2\2CD\3\2\2\2DB\3\2\2"+
		"\2DE\3\2\2\2E\6\3\2\2\2FG\7\62\2\2GH\7z\2\2HJ\3\2\2\2IK\t\4\2\2JI\3\2"+
		"\2\2KL\3\2\2\2LJ\3\2\2\2LM\3\2\2\2M\b\3\2\2\2Nc\5\17\b\2OQ\5)\25\2PO\3"+
		"\2\2\2QT\3\2\2\2RP\3\2\2\2RS\3\2\2\2SU\3\2\2\2TR\3\2\2\2U`\5\33\16\2V"+
		"X\5)\25\2WV\3\2\2\2X[\3\2\2\2YW\3\2\2\2YZ\3\2\2\2Z\\\3\2\2\2[Y\3\2\2\2"+
		"\\]\t\5\2\2]_\5\33\16\2^Y\3\2\2\2_b\3\2\2\2`^\3\2\2\2`a\3\2\2\2ad\3\2"+
		"\2\2b`\3\2\2\2cR\3\2\2\2cd\3\2\2\2d\n\3\2\2\2eg\t\3\2\2fe\3\2\2\2fg\3"+
		"\2\2\2gh\3\2\2\2hi\7q\2\2iq\7q\2\2jl\5)\25\2kj\3\2\2\2lm\3\2\2\2mk\3\2"+
		"\2\2mn\3\2\2\2no\3\2\2\2op\5\33\16\2pr\3\2\2\2qk\3\2\2\2qr\3\2\2\2r\f"+
		"\3\2\2\2s\u0084\5\17\b\2tv\5)\25\2ut\3\2\2\2vy\3\2\2\2wu\3\2\2\2wx\3\2"+
		"\2\2xz\3\2\2\2yw\3\2\2\2z~\t\3\2\2{}\5)\25\2|{\3\2\2\2}\u0080\3\2\2\2"+
		"~|\3\2\2\2~\177\3\2\2\2\177\u0081\3\2\2\2\u0080~\3\2\2\2\u0081\u0082\5"+
		"\21\t\2\u0082\u0083\7k\2\2\u0083\u0085\3\2\2\2\u0084w\3\2\2\2\u0084\u0085"+
		"\3\2\2\2\u0085\16\3\2\2\2\u0086\u0088\7/\2\2\u0087\u0086\3\2\2\2\u0087"+
		"\u0088\3\2\2\2\u0088\u0089\3\2\2\2\u0089\u008a\5\21\t\2\u008a\20\3\2\2"+
		"\2\u008b\u008f\5\23\n\2\u008c\u008e\5)\25\2\u008d\u008c\3\2\2\2\u008e"+
		"\u0091\3\2\2\2\u008f\u008d\3\2\2\2\u008f\u0090\3\2\2\2\u0090\u0092\3\2"+
		"\2\2\u0091\u008f\3\2\2\2\u0092\u0096\7\61\2\2\u0093\u0095\5)\25\2\u0094"+
		"\u0093\3\2\2\2\u0095\u0098\3\2\2\2\u0096\u0094\3\2\2\2\u0096\u0097\3\2"+
		"\2\2\u0097\u0099\3\2\2\2\u0098\u0096\3\2\2\2\u0099\u009a\5\23\n\2\u009a"+
		"\u00ae\3\2\2\2\u009b\u00a2\5\23\n\2\u009c\u009e\7\60\2\2\u009d\u009f\4"+
		"\62;\2\u009e\u009d\3\2\2\2\u009f\u00a0\3\2\2\2\u00a0\u009e\3\2\2\2\u00a0"+
		"\u00a1\3\2\2\2\u00a1\u00a3\3\2\2\2\u00a2\u009c\3\2\2\2\u00a2\u00a3\3\2"+
		"\2\2\u00a3\u00ae\3\2\2\2\u00a4\u00ab\7\62\2\2\u00a5\u00a7\7\60\2\2\u00a6"+
		"\u00a8\4\62;\2\u00a7\u00a6\3\2\2\2\u00a8\u00a9\3\2\2\2\u00a9\u00a7\3\2"+
		"\2\2\u00a9\u00aa\3\2\2\2\u00aa\u00ac\3\2\2\2\u00ab\u00a5\3\2\2\2\u00ab"+
		"\u00ac\3\2\2\2\u00ac\u00ae\3\2\2\2\u00ad\u008b\3\2\2\2\u00ad\u009b\3\2"+
		"\2\2\u00ad\u00a4\3\2\2\2\u00ae\22\3\2\2\2\u00af\u00b2\5\25\13\2\u00b0"+
		"\u00b2\5\27\f\2\u00b1\u00af\3\2\2\2\u00b1\u00b0\3\2\2\2\u00b2\24\3\2\2"+
		"\2\u00b3\u00b7\4\63;\2\u00b4\u00b6\4\62;\2\u00b5\u00b4\3\2\2\2\u00b6\u00b9"+
		"\3\2\2\2\u00b7\u00b5\3\2\2\2\u00b7\u00b8\3\2\2\2\u00b8\26\3\2\2\2\u00b9"+
		"\u00b7\3\2\2\2\u00ba\u00bc\4\63;\2\u00bb\u00bd\4\62;\2\u00bc\u00bb\3\2"+
		"\2\2\u00bc\u00bd\3\2\2\2\u00bd\u00bf\3\2\2\2\u00be\u00c0\4\62;\2\u00bf"+
		"\u00be\3\2\2\2\u00bf\u00c0\3\2\2\2\u00c0\u00c2\3\2\2\2\u00c1\u00c3\5\31"+
		"\r\2\u00c2\u00c1\3\2\2\2\u00c3\u00c4\3\2\2\2\u00c4\u00c2\3\2\2\2\u00c4"+
		"\u00c5\3\2\2\2\u00c5\30\3\2\2\2\u00c6\u00c7\7)\2\2\u00c7\u00c8\4\62;\2"+
		"\u00c8\u00c9\4\62;\2\u00c9\u00ca\4\62;\2\u00ca\32\3\2\2\2\u00cb\u00cf"+
		"\5!\21\2\u00cc\u00cf\5\35\17\2\u00cd\u00cf\5\37\20\2\u00ce\u00cb\3\2\2"+
		"\2\u00ce\u00cc\3\2\2\2\u00ce\u00cd\3\2\2\2\u00cf\34\3\2\2\2\u00d0\u00d1"+
		"\7v\2\2\u00d1\u010d\7j\2\2\u00d2\u00d3\7k\2\2\u00d3\u010d\7p\2\2\u00d4"+
		"\u00d5\7h\2\2\u00d5\u010d\7v\2\2\u00d6\u00d7\7{\2\2\u00d7\u010d\7f\2\2"+
		"\u00d8\u00d9\7e\2\2\u00d9\u010d\7j\2\2\u00da\u00db\7h\2\2\u00db\u00dc"+
		"\7w\2\2\u00dc\u010d\7t\2\2\u00dd\u00de\7o\2\2\u00de\u010d\7n\2\2\u00df"+
		"\u00e0\7n\2\2\u00e0\u00e1\7g\2\2\u00e1\u010d\7c\2\2\u00e2\u00e3\7h\2\2"+
		"\u00e3\u00e4\7v\2\2\u00e4\u010d\7o\2\2\u00e5\u00e6\7h\2\2\u00e6\u00e7"+
		"\7n\2\2\u00e7\u00e8\7\"\2\2\u00e8\u00e9\7q\2\2\u00e9\u010d\7|\2\2\u00ea"+
		"\u00eb\7i\2\2\u00eb\u010d\7k\2\2\u00ec\u00ed\7r\2\2\u00ed\u010d\7v\2\2"+
		"\u00ee\u00ef\7s\2\2\u00ef\u010d\7v\2\2\u00f0\u00f1\7i\2\2\u00f1\u00f2"+
		"\7c\2\2\u00f2\u010d\7n\2\2\u00f3\u00f4\7i\2\2\u00f4\u010d\7t\2\2\u00f5"+
		"\u00f6\7f\2\2\u00f6\u010d\7t\2\2\u00f7\u00f8\7q\2\2\u00f8\u010d\7|\2\2"+
		"\u00f9\u00fa\7n\2\2\u00fa\u010d\7d\2\2\u00fb\u00fc\7u\2\2\u00fc\u010d"+
		"\7v\2\2\u00fd\u00fe\7s\2\2\u00fe\u010d\7t\2\2\u00ff\u0100\7s\2\2\u0100"+
		"\u0101\7v\2\2\u0101\u010d\7t\2\2\u0102\u0103\7e\2\2\u0103\u0104\7y\2\2"+
		"\u0104\u010d\7v\2\2\u0105\u0106\7u\2\2\u0106\u0107\7n\2\2\u0107\u0108"+
		"\7w\2\2\u0108\u010d\7i\2\2\u0109\u010a\7\u00c4\2\2\u010a\u010b\7\u00b2"+
		"\2\2\u010b\u010d\7H\2\2\u010c\u00d0\3\2\2\2\u010c\u00d2\3\2\2\2\u010c"+
		"\u00d4\3\2\2\2\u010c\u00d6\3\2\2\2\u010c\u00d8\3\2\2\2\u010c\u00da\3\2"+
		"\2\2\u010c\u00dd\3\2\2\2\u010c\u00df\3\2\2\2\u010c\u00e2\3\2\2\2\u010c"+
		"\u00e5\3\2\2\2\u010c\u00ea\3\2\2\2\u010c\u00ec\3\2\2\2\u010c\u00ee\3\2"+
		"\2\2\u010c\u00f0\3\2\2\2\u010c\u00f3\3\2\2\2\u010c\u00f5\3\2\2\2\u010c"+
		"\u00f7\3\2\2\2\u010c\u00f9\3\2\2\2\u010c\u00fb\3\2\2\2\u010c\u00fd\3\2"+
		"\2\2\u010c\u00ff\3\2\2\2\u010c\u0102\3\2\2\2\u010c\u0105\3\2\2\2\u010c"+
		"\u0109\3\2\2\2\u010d\36\3\2\2\2\u010e\u010f\7o\2\2\u010f\u0110\7k\2\2"+
		"\u0110\u0122\7p\2\2\u0111\u0122\t\6\2\2\u0112\u0113\7j\2\2\u0113\u0122"+
		"\7c\2\2\u0114\u0122\t\7\2\2\u0115\u0116\7c\2\2\u0116\u0122\7w\2\2\u0117"+
		"\u0118\7C\2\2\u0118\u0122\7W\2\2\u0119\u011a\7P\2\2\u011a\u0122\7r\2\2"+
		"\u011b\u0122\7D\2\2\u011c\u011d\7f\2\2\u011d\u0122\7D\2\2\u011e\u011f"+
		"\7g\2\2\u011f\u0122\7X\2\2\u0120\u0122\7w\2\2\u0121\u010e\3\2\2\2\u0121"+
		"\u0111\3\2\2\2\u0121\u0112\3\2\2\2\u0121\u0114\3\2\2\2\u0121\u0115\3\2"+
		"\2\2\u0121\u0117\3\2\2\2\u0121\u0119\3\2\2\2\u0121\u011b\3\2\2\2\u0121"+
		"\u011c\3\2\2\2\u0121\u011e\3\2\2\2\u0121\u0120\3\2\2\2\u0122 \3\2\2\2"+
		"\u0123\u0125\5#\22\2\u0124\u0123\3\2\2\2\u0124\u0125\3\2\2\2\u0125\u0126"+
		"\3\2\2\2\u0126\u012c\5%\23\2\u0127\u0129\7`\2\2\u0128\u012a\7/\2\2\u0129"+
		"\u0128\3\2\2\2\u0129\u012a\3\2\2\2\u012a\u012b\3\2\2\2\u012b\u012d\4\62"+
		";\2\u012c\u0127\3\2\2\2\u012c\u012d\3\2\2\2\u012d\u013d\3\2\2\2\u012e"+
		"\u0130\t\5\2\2\u012f\u0131\5#\22\2\u0130\u012f\3\2\2\2\u0130\u0131\3\2"+
		"\2\2\u0131\u0132\3\2\2\2\u0132\u0133\5%\23\2\u0133\u0139\3\2\2\2\u0134"+
		"\u0136\7`\2\2\u0135\u0137\7/\2\2\u0136\u0135\3\2\2\2\u0136\u0137\3\2\2"+
		"\2\u0137\u0138\3\2\2\2\u0138\u013a\4\62;\2\u0139\u0134\3\2\2\2\u0139\u013a"+
		"\3\2\2\2\u013a\u013c\3\2\2\2\u013b\u012e\3\2\2\2\u013c\u013f\3\2\2\2\u013d"+
		"\u013b\3\2\2\2\u013d\u013e\3\2\2\2\u013e\u0142\3\2\2\2\u013f\u013d\3\2"+
		"\2\2\u0140\u0142\5\'\24\2\u0141\u0124\3\2\2\2\u0141\u0140\3\2\2\2\u0142"+
		"\"\3\2\2\2\u0143\u0148\t\b\2\2\u0144\u0145\7f\2\2\u0145\u0148\7c\2\2\u0146"+
		"\u0148\t\t\2\2\u0147\u0143\3\2\2\2\u0147\u0144\3\2\2\2\u0147\u0146\3\2"+
		"\2\2\u0148$\3\2\2\2\u0149\u016c\t\n\2\2\u014a\u014b\7o\2\2\u014b\u014c"+
		"\7q\2\2\u014c\u016c\7n\2\2\u014d\u014e\7e\2\2\u014e\u016c\7f\2\2\u014f"+
		"\u0150\7J\2\2\u0150\u016c\7|\2\2\u0151\u016c\7P\2\2\u0152\u0153\7R\2\2"+
		"\u0153\u016c\7c\2\2\u0154\u016c\t\13\2\2\u0155\u0156\7\u00d0\2\2\u0156"+
		"\u016c\7\u00ab\2\2\u0157\u016c\7U\2\2\u0158\u0159\7Y\2\2\u0159\u016c\7"+
		"d\2\2\u015a\u016c\t\f\2\2\u015b\u015c\7\u00c4\2\2\u015c\u015d\7\u00b2"+
		"\2\2\u015d\u016c\7E\2\2\u015e\u015f\7n\2\2\u015f\u016c\7o\2\2\u0160\u0161"+
		"\7n\2\2\u0161\u016c\7z\2\2\u0162\u0163\7D\2\2\u0163\u016c\7s\2\2\u0164"+
		"\u0165\7I\2\2\u0165\u016c\7{\2\2\u0166\u0167\7U\2\2\u0167\u016c\7x\2\2"+
		"\u0168\u0169\7m\2\2\u0169\u016a\7c\2\2\u016a\u016c\7v\2\2\u016b\u0149"+
		"\3\2\2\2\u016b\u014a\3\2\2\2\u016b\u014d\3\2\2\2\u016b\u014f\3\2\2\2\u016b"+
		"\u0151\3\2\2\2\u016b\u0152\3\2\2\2\u016b\u0154\3\2\2\2\u016b\u0155\3\2"+
		"\2\2\u016b\u0157\3\2\2\2\u016b\u0158\3\2\2\2\u016b\u015a\3\2\2\2\u016b"+
		"\u015b\3\2\2\2\u016b\u015e\3\2\2\2\u016b\u0160\3\2\2\2\u016b\u0162\3\2"+
		"\2\2\u016b\u0164\3\2\2\2\u016b\u0166\3\2\2\2\u016b\u0168\3\2\2\2\u016c"+
		"&\3\2\2\2\u016d\u016e\7t\2\2\u016e\u016f\7c\2\2\u016f\u0173\7f\2\2\u0170"+
		"\u0171\7u\2\2\u0171\u0173\7t\2\2\u0172\u016d\3\2\2\2\u0172\u0170\3\2\2"+
		"\2\u0173(\3\2\2\2\u0174\u0175\t\r\2\2\u0175*\3\2\2\2\u0176\u0177\t\16"+
		"\2\2\u0177,\3\2\2\2\u0178\u0179\t\17\2\2\u0179\u0184\5+\26\2\u017a\u017b"+
		"\t\20\2\2\u017b\u0184\t\21\2\2\u017c\u017d\t\20\2\2\u017d\u017f\5+\26"+
		"\2\u017e\u0180\5+\26\2\u017f\u017e\3\2\2\2\u0180\u0181\3\2\2\2\u0181\u017f"+
		"\3\2\2\2\u0181\u0182\3\2\2\2\u0182\u0184\3\2\2\2\u0183\u0178\3\2\2\2\u0183"+
		"\u017a\3\2\2\2\u0183\u017c\3\2\2\2\u0184.\3\2\2\2\u0185\u0186\7\17\2\2"+
		"\u0186\u0189\7\f\2\2\u0187\u0189\t\22\2\2\u0188\u0185\3\2\2\2\u0188\u0187"+
		"\3\2\2\2\u0189\60\3\2\2\2\u018a\u018f\t\r\2\2\u018b\u018c\7\17\2\2\u018c"+
		"\u018f\7\f\2\2\u018d\u018f\t\22\2\2\u018e\u018a\3\2\2\2\u018e\u018b\3"+
		"\2\2\2\u018e\u018d\3\2\2\2\u018f\u0190\3\2\2\2\u0190\u0191\b\31\2\2\u0191"+
		"\62\3\2\2\2\u0192\u0193\7\61\2\2\u0193\u0194\7\61\2\2\u0194\u0198\3\2"+
		"\2\2\u0195\u0197\n\22\2\2\u0196\u0195\3\2\2\2\u0197\u019a\3\2\2\2\u0198"+
		"\u0196\3\2\2\2\u0198\u0199\3\2\2\2\u0199\u01a0\3\2\2\2\u019a\u0198\3\2"+
		"\2\2\u019b\u01a1\7\f\2\2\u019c\u019e\7\17\2\2\u019d\u019f\7\f\2\2\u019e"+
		"\u019d\3\2\2\2\u019e\u019f\3\2\2\2\u019f\u01a1\3\2\2\2\u01a0\u019b\3\2"+
		"\2\2\u01a0\u019c\3\2\2\2\u01a0\u01a1\3\2\2\2\u01a1\u01a2\3\2\2\2\u01a2"+
		"\u01a3\b\32\3\2\u01a3\64\3\2\2\2\u01a4\u01a5\7\61\2\2\u01a5\u01a6\7,\2"+
		"\2\u01a6\u01ab\3\2\2\2\u01a7\u01aa\5/\30\2\u01a8\u01aa\n\23\2\2\u01a9"+
		"\u01a7\3\2\2\2\u01a9\u01a8\3\2\2\2\u01aa\u01ad\3\2\2\2\u01ab\u01a9\3\2"+
		"\2\2\u01ab\u01ac\3\2\2\2\u01ac\u01ae\3\2\2\2\u01ad\u01ab\3\2\2\2\u01ae"+
		"\u01af\7,\2\2\u01af\u01b0\7\61\2\2\u01b0\u01b1\3\2\2\2\u01b1\u01b2\b\33"+
		"\4\2\u01b2\66\3\2\2\2\64\2=DLRY`cfmqw~\u0084\u0087\u008f\u0096\u00a0\u00a2"+
		"\u00a9\u00ab\u00ad\u00b1\u00b7\u00bc\u00bf\u00c4\u00ce\u010c\u0121\u0124"+
		"\u0129\u012c\u0130\u0136\u0139\u013d\u0141\u0147\u016b\u0172\u0181\u0183"+
		"\u0188\u018e\u0198\u019e\u01a0\u01a9\u01ab\5\3\31\2\3\32\3\3\33\4";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}