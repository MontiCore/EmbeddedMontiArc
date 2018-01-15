// Generated from C:\Users\Delta2-PC\Desktop\Uni\SimLang\out\weather\_parser\WeatherAntlr.g4 by ANTLR 4.5.1

package weather._parser;
import de.monticore.antlr4.MCParser;

import org.antlr.v4.runtime.atn.*;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.misc.*;
import org.antlr.v4.runtime.tree.*;
import java.util.List;
import java.util.Iterator;
import java.util.ArrayList;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast"})
public class WeatherAntlrParser extends MCParser {
	static { RuntimeMetaData.checkVersion("4.5.1", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		T__0=1, T__1=2, T__2=3, T__3=4, T__4=5, T__5=6, T__6=7, T__7=8, T__8=9, 
		T__9=10, T__10=11, T__11=12, T__12=13, T__13=14, T__14=15, T__15=16, T__16=17, 
		T__17=18, T__18=19, T__19=20, T__20=21, T__21=22, T__22=23, T__23=24, 
		T__24=25, T__25=26, T__26=27, T__27=28, CONTRAILS=29, SNAIN=30, ALTOSTRATUS=31, 
		SMOG=32, LANDSPOUT=33, NONE=34, NIMBOSTRATUS=35, CUMULONIMBUS=36, STRATUS=37, 
		STRATOCUMULUS=38, MIRAGE=39, TEMPERATURE=40, SIGHT=41, HUMIDITY=42, WATERSPOUT=43, 
		SLEET=44, NOCTILUCENT=45, GRAUPEL=46, RAIN=47, ALTOCUMULUS=48, PERCENT=49, 
		UNLIMITED=50, CIRRUS=51, GUSTNADO=52, LPAREN=53, RPAREN=54, PRESSURE=55, 
		DRIZZLE=56, HAIL=57, COMMA=58, WINDDIRECTION=59, CIRROCUMULUS=60, RAINBOW=61, 
		CLOUDING=62, SNOW=63, WINDSTRENGTH=64, CIRROSTRATUS=65, THUNDERSTORM=66, 
		FOG=67, TFloatPointUnitNumber=68, THexUnitNumber=69, TUnitNumber=70, TUnitInf=71, 
		TComplexNumber=72, Name=73, WS=74, SL_COMMENT=75, ML_COMMENT=76;
	public static final int
		RULE_number_eof = 0, RULE_number = 1, RULE_floatPointUnitNumber_eof = 2, 
		RULE_floatPointUnitNumber = 3, RULE_hexUnitNumber_eof = 4, RULE_hexUnitNumber = 5, 
		RULE_unitNumber_eof = 6, RULE_unitNumber = 7, RULE_complexNumber_eof = 8, 
		RULE_complexNumber = 9, RULE_temperature_eof = 10, RULE_temperature = 11, 
		RULE_humidity_eof = 12, RULE_humidity = 13, RULE_pressure_eof = 14, RULE_pressure = 15, 
		RULE_windstrength_eof = 16, RULE_windstrength = 17, RULE_winddirection_eof = 18, 
		RULE_winddirection = 19, RULE_precipitationtype_eof = 20, RULE_precipitationtype = 21, 
		RULE_precipitationamount_eof = 22, RULE_precipitationamount = 23, RULE_clouding_eof = 24, 
		RULE_clouding = 25, RULE_sight_eof = 26, RULE_sight = 27, RULE_weatherPhenomena_eof = 28, 
		RULE_weatherPhenomena = 29, RULE_opticalPhenomena_eof = 30, RULE_opticalPhenomena = 31, 
		RULE_artificialPhenomena_eof = 32, RULE_artificialPhenomena = 33;
	public static final String[] ruleNames = {
		"number_eof", "number", "floatPointUnitNumber_eof", "floatPointUnitNumber", 
		"hexUnitNumber_eof", "hexUnitNumber", "unitNumber_eof", "unitNumber", 
		"complexNumber_eof", "complexNumber", "temperature_eof", "temperature", 
		"humidity_eof", "humidity", "pressure_eof", "pressure", "windstrength_eof", 
		"windstrength", "winddirection_eof", "winddirection", "precipitationtype_eof", 
		"precipitationtype", "precipitationamount_eof", "precipitationamount", 
		"clouding_eof", "clouding", "sight_eof", "sight", "weatherPhenomena_eof", 
		"weatherPhenomena", "opticalPhenomena_eof", "opticalPhenomena", "artificialPhenomena_eof", 
		"artificialPhenomena"
	};

	private static final String[] _LITERAL_NAMES = {
		null, "'precipitation_type'", "'freezing drizzle'", "'freezing rain'", 
		"'snow rain'", "'snow_grains'", "'ice_pellets'", "'snow_pellets'", "'ice_crystals'", 
		"'precipitation_amount'", "'polar stratospheric'", "'cumulus_humilis'", 
		"'cumulus_mediocris'", "'cumulus_congestus'", "'weather_phenomena'", "'rope_tornado'", 
		"'cone_tornado'", "'wedge_tornado'", "'multi-vortex_tornado'", "'dust_devil'", 
		"'steam_devil'", "'optical_phenomena'", "'northern_lights'", "'circumzenithal_arc'", 
		"'zodiacal_light'", "'crepuscular_rays'", "'fog_bow'", "'artificial_phenomena'", 
		"'rocket_exhaust_trails'", "'contrails'", "'snain'", "'altostratus'", 
		"'smog'", "'landspout'", "'none'", "'nimbostratus'", "'cumulonimbus'", 
		"'stratus'", "'stratocumulus'", "'mirage'", "'temperature'", "'sight'", 
		"'humidity'", "'waterspout'", "'sleet'", "'noctilucent'", "'graupel'", 
		"'rain'", "'altocumulus'", "'%'", "'unlimited'", "'cirrus'", "'gustnado'", 
		"'('", "')'", "'pressure'", "'drizzle'", "'hail'", "','", "'winddirection'", 
		"'cirrocumulus'", "'rainbow'", "'clouding'", "'snow'", "'windstrength'", 
		"'cirrostratus'", "'thunderstorm'", "'fog'"
	};
	private static final String[] _SYMBOLIC_NAMES = {
		null, null, null, null, null, null, null, null, null, null, null, null, 
		null, null, null, null, null, null, null, null, null, null, null, null, 
		null, null, null, null, null, "CONTRAILS", "SNAIN", "ALTOSTRATUS", "SMOG", 
		"LANDSPOUT", "NONE", "NIMBOSTRATUS", "CUMULONIMBUS", "STRATUS", "STRATOCUMULUS", 
		"MIRAGE", "TEMPERATURE", "SIGHT", "HUMIDITY", "WATERSPOUT", "SLEET", "NOCTILUCENT", 
		"GRAUPEL", "RAIN", "ALTOCUMULUS", "PERCENT", "UNLIMITED", "CIRRUS", "GUSTNADO", 
		"LPAREN", "RPAREN", "PRESSURE", "DRIZZLE", "HAIL", "COMMA", "WINDDIRECTION", 
		"CIRROCUMULUS", "RAINBOW", "CLOUDING", "SNOW", "WINDSTRENGTH", "CIRROSTRATUS", 
		"THUNDERSTORM", "FOG", "TFloatPointUnitNumber", "THexUnitNumber", "TUnitNumber", 
		"TUnitInf", "TComplexNumber", "Name", "WS", "SL_COMMENT", "ML_COMMENT"
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

	@Override
	public String getGrammarFileName() { return "WeatherAntlr.g4"; }

	@Override
	public String[] getRuleNames() { return ruleNames; }

	@Override
	public String getSerializedATN() { return _serializedATN; }

	@Override
	public ATN getATN() { return _ATN; }


	// Global actions

	// Convert functions
	  // convert function for TFloatPointUnitNumber
	private String convertTFloatPointUnitNumber(Token t)  {
	    return t.getText();
	}

	  // convert function for Significant
	private String convertSignificant(Token t)  {
	    return t.getText();
	}

	  // convert function for THexUnitNumber
	private String convertTHexUnitNumber(Token t)  {
	    return t.getText();
	}

	  // convert function for TUnitNumber
	private String convertTUnitNumber(Token t)  {
	    return t.getText();
	}

	  // convert function for TUnitInf
	private String convertTUnitInf(Token t)  {
	    return t.getText();
	}

	  // convert function for TComplexNumber
	private String convertTComplexNumber(Token t)  {
	    return t.getText();
	}

	  // convert function for RealNumber
	private String convertRealNumber(Token t)  {
	    return t.getText();
	}

	  // convert function for PosNumber
	private String convertPosNumber(Token t)  {
	    return t.getText();
	}

	  // convert function for PosInt
	private String convertPosInt(Token t)  {
	    return t.getText();
	}

	  // convert function for UngroupedPosInt
	private String convertUngroupedPosInt(Token t)  {
	    return t.getText();
	}

	  // convert function for GroupedPosInt
	private String convertGroupedPosInt(Token t)  {
	    return t.getText();
	}

	  // convert function for PosIntGroup
	private String convertPosIntGroup(Token t)  {
	    return t.getText();
	}

	  // convert function for Unit
	private String convertUnit(Token t)  {
	    return t.getText();
	}

	  // convert function for ImperialUnit
	private String convertImperialUnit(Token t)  {
	    return t.getText();
	}

	  // convert function for SIUnit
	private String convertSIUnit(Token t)  {
	    return t.getText();
	}

	  // convert function for UnitPrefix
	private String convertUnitPrefix(Token t)  {
	    return t.getText();
	}

	  // convert function for OfficallyAcceptedUnit
	private String convertOfficallyAcceptedUnit(Token t)  {
	    return t.getText();
	}

	  // convert function for SiUnitBaseDimension
	private String convertSiUnitBaseDimension(Token t)  {
	    return t.getText();
	}

	  // convert function for SiUnitDimensionless
	private String convertSiUnitDimensionless(Token t)  {
	    return t.getText();
	}

	  // convert function for Space
	private String convertSpace(Token t)  {
	    return t.getText();
	}

	  // convert function for NamePart
	private String convertNamePart(Token t)  {
	    return t.getText();
	}

	  // convert function for Name
	private String convertName(Token t)  {
	    return t.getText();
	}

	  // convert function for NEWLINE
	private String convertNEWLINE(Token t)  {
	    return t.getText();
	}

	  // convert function for WS
	private String convertWS(Token t)  {
	    return t.getText();
	}

	  // convert function for SL_COMMENT
	private String convertSL_COMMENT(Token t)  {
	    return t.getText();
	}

	  // convert function for ML_COMMENT
	private String convertML_COMMENT(Token t)  {
	    return t.getText();
	}


	public WeatherAntlrParser(TokenStream input) {
		super(input);
		_interp = new ParserATNSimulator(this,_ATN,_decisionToDFA,_sharedContextCache);
	}
	public static class Number_eofContext extends ParserRuleContext {
		public numberunit._ast.ASTNumber ret =  null;
		public NumberContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public NumberContext number() {
			return getRuleContext(NumberContext.class,0);
		}
		public Number_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_number_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitNumber_eof(this);
		}
	}

	public final Number_eofContext number_eof() throws RecognitionException {
		Number_eofContext _localctx = new Number_eofContext(_ctx, getState());
		enterRule(_localctx, 0, RULE_number_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(68);
			((Number_eofContext)_localctx).tmp = number();
			((Number_eofContext)_localctx).ret =  ((Number_eofContext)_localctx).tmp.ret;
			setState(70);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class NumberContext extends ParserRuleContext {
		public numberunit._ast.ASTNumber ret =  null;
		public FloatPointUnitNumberContext tmp0;
		public HexUnitNumberContext tmp1;
		public ComplexNumberContext tmp2;
		public UnitNumberContext tmp3;
		public FloatPointUnitNumberContext floatPointUnitNumber() {
			return getRuleContext(FloatPointUnitNumberContext.class,0);
		}
		public HexUnitNumberContext hexUnitNumber() {
			return getRuleContext(HexUnitNumberContext.class,0);
		}
		public ComplexNumberContext complexNumber() {
			return getRuleContext(ComplexNumberContext.class,0);
		}
		public UnitNumberContext unitNumber() {
			return getRuleContext(UnitNumberContext.class,0);
		}
		public NumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_number; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitNumber(this);
		}
	}

	public final NumberContext number() throws RecognitionException {
		NumberContext _localctx = new NumberContext(_ctx, getState());
		enterRule(_localctx, 2, RULE_number);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		numberunit._ast.ASTNumber _aNode = null;
		_aNode=numberunit._ast.NumberUnitNodeFactory.createASTNumber();
		((NumberContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			setState(84);
			switch (_input.LA(1)) {
			case TFloatPointUnitNumber:
				enterOuterAlt(_localctx, 1);
				{
				setState(72);
				((NumberContext)_localctx).tmp0 = floatPointUnitNumber();
				_aNode.setFloatPointUnitNumber(_localctx.tmp0.ret);
				}
				break;
			case THexUnitNumber:
				enterOuterAlt(_localctx, 2);
				{
				setState(75);
				((NumberContext)_localctx).tmp1 = hexUnitNumber();
				_aNode.setHexUnitNumber(_localctx.tmp1.ret);
				}
				break;
			case TComplexNumber:
				enterOuterAlt(_localctx, 3);
				{
				setState(78);
				((NumberContext)_localctx).tmp2 = complexNumber();
				_aNode.setComplexNumber(_localctx.tmp2.ret);
				}
				break;
			case TUnitNumber:
			case TUnitInf:
				enterOuterAlt(_localctx, 4);
				{
				setState(81);
				((NumberContext)_localctx).tmp3 = unitNumber();
				_aNode.setUnitNumber(_localctx.tmp3.ret);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class FloatPointUnitNumber_eofContext extends ParserRuleContext {
		public numberunit._ast.ASTFloatPointUnitNumber ret =  null;
		public FloatPointUnitNumberContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public FloatPointUnitNumberContext floatPointUnitNumber() {
			return getRuleContext(FloatPointUnitNumberContext.class,0);
		}
		public FloatPointUnitNumber_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_floatPointUnitNumber_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterFloatPointUnitNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitFloatPointUnitNumber_eof(this);
		}
	}

	public final FloatPointUnitNumber_eofContext floatPointUnitNumber_eof() throws RecognitionException {
		FloatPointUnitNumber_eofContext _localctx = new FloatPointUnitNumber_eofContext(_ctx, getState());
		enterRule(_localctx, 4, RULE_floatPointUnitNumber_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(86);
			((FloatPointUnitNumber_eofContext)_localctx).tmp = floatPointUnitNumber();
			((FloatPointUnitNumber_eofContext)_localctx).ret =  ((FloatPointUnitNumber_eofContext)_localctx).tmp.ret;
			setState(88);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class FloatPointUnitNumberContext extends ParserRuleContext {
		public numberunit._ast.ASTFloatPointUnitNumber ret =  null;
		public Token tmp0;
		public TerminalNode TFloatPointUnitNumber() { return getToken(WeatherAntlrParser.TFloatPointUnitNumber, 0); }
		public FloatPointUnitNumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_floatPointUnitNumber; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterFloatPointUnitNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitFloatPointUnitNumber(this);
		}
	}

	public final FloatPointUnitNumberContext floatPointUnitNumber() throws RecognitionException {
		FloatPointUnitNumberContext _localctx = new FloatPointUnitNumberContext(_ctx, getState());
		enterRule(_localctx, 6, RULE_floatPointUnitNumber);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		numberunit._ast.ASTFloatPointUnitNumber _aNode = null;
		_aNode=numberunit._ast.NumberUnitNodeFactory.createASTFloatPointUnitNumber();
		((FloatPointUnitNumberContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(90);
			((FloatPointUnitNumberContext)_localctx).tmp0 = match(TFloatPointUnitNumber);
			_aNode.setTFloatPointUnitNumber(convertTFloatPointUnitNumber(((FloatPointUnitNumberContext)_localctx).tmp0));
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class HexUnitNumber_eofContext extends ParserRuleContext {
		public numberunit._ast.ASTHexUnitNumber ret =  null;
		public HexUnitNumberContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public HexUnitNumberContext hexUnitNumber() {
			return getRuleContext(HexUnitNumberContext.class,0);
		}
		public HexUnitNumber_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_hexUnitNumber_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterHexUnitNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitHexUnitNumber_eof(this);
		}
	}

	public final HexUnitNumber_eofContext hexUnitNumber_eof() throws RecognitionException {
		HexUnitNumber_eofContext _localctx = new HexUnitNumber_eofContext(_ctx, getState());
		enterRule(_localctx, 8, RULE_hexUnitNumber_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(93);
			((HexUnitNumber_eofContext)_localctx).tmp = hexUnitNumber();
			((HexUnitNumber_eofContext)_localctx).ret =  ((HexUnitNumber_eofContext)_localctx).tmp.ret;
			setState(95);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class HexUnitNumberContext extends ParserRuleContext {
		public numberunit._ast.ASTHexUnitNumber ret =  null;
		public Token tmp0;
		public TerminalNode THexUnitNumber() { return getToken(WeatherAntlrParser.THexUnitNumber, 0); }
		public HexUnitNumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_hexUnitNumber; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterHexUnitNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitHexUnitNumber(this);
		}
	}

	public final HexUnitNumberContext hexUnitNumber() throws RecognitionException {
		HexUnitNumberContext _localctx = new HexUnitNumberContext(_ctx, getState());
		enterRule(_localctx, 10, RULE_hexUnitNumber);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		numberunit._ast.ASTHexUnitNumber _aNode = null;
		_aNode=numberunit._ast.NumberUnitNodeFactory.createASTHexUnitNumber();
		((HexUnitNumberContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(97);
			((HexUnitNumberContext)_localctx).tmp0 = match(THexUnitNumber);
			_aNode.setTHexUnitNumber(convertTHexUnitNumber(((HexUnitNumberContext)_localctx).tmp0));
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class UnitNumber_eofContext extends ParserRuleContext {
		public numberunit._ast.ASTUnitNumber ret =  null;
		public UnitNumberContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public UnitNumberContext unitNumber() {
			return getRuleContext(UnitNumberContext.class,0);
		}
		public UnitNumber_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unitNumber_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterUnitNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitUnitNumber_eof(this);
		}
	}

	public final UnitNumber_eofContext unitNumber_eof() throws RecognitionException {
		UnitNumber_eofContext _localctx = new UnitNumber_eofContext(_ctx, getState());
		enterRule(_localctx, 12, RULE_unitNumber_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(100);
			((UnitNumber_eofContext)_localctx).tmp = unitNumber();
			((UnitNumber_eofContext)_localctx).ret =  ((UnitNumber_eofContext)_localctx).tmp.ret;
			setState(102);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class UnitNumberContext extends ParserRuleContext {
		public numberunit._ast.ASTUnitNumber ret =  null;
		public Token tmp0;
		public Token tmp1;
		public TerminalNode TUnitNumber() { return getToken(WeatherAntlrParser.TUnitNumber, 0); }
		public TerminalNode TUnitInf() { return getToken(WeatherAntlrParser.TUnitInf, 0); }
		public UnitNumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unitNumber; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterUnitNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitUnitNumber(this);
		}
	}

	public final UnitNumberContext unitNumber() throws RecognitionException {
		UnitNumberContext _localctx = new UnitNumberContext(_ctx, getState());
		enterRule(_localctx, 14, RULE_unitNumber);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		numberunit._ast.ASTUnitNumber _aNode = null;
		_aNode=numberunit._ast.NumberUnitNodeFactory.createASTUnitNumber();
		((UnitNumberContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			setState(108);
			switch (_input.LA(1)) {
			case TUnitNumber:
				enterOuterAlt(_localctx, 1);
				{
				{
				setState(104);
				((UnitNumberContext)_localctx).tmp0 = match(TUnitNumber);
				_aNode.setTUnitNumber(convertTUnitNumber(((UnitNumberContext)_localctx).tmp0));
				}
				}
				break;
			case TUnitInf:
				enterOuterAlt(_localctx, 2);
				{
				{
				setState(106);
				((UnitNumberContext)_localctx).tmp1 = match(TUnitInf);
				_aNode.setTUnitInf(convertTUnitInf(((UnitNumberContext)_localctx).tmp1));
				}
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class ComplexNumber_eofContext extends ParserRuleContext {
		public numberunit._ast.ASTComplexNumber ret =  null;
		public ComplexNumberContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public ComplexNumberContext complexNumber() {
			return getRuleContext(ComplexNumberContext.class,0);
		}
		public ComplexNumber_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_complexNumber_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterComplexNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitComplexNumber_eof(this);
		}
	}

	public final ComplexNumber_eofContext complexNumber_eof() throws RecognitionException {
		ComplexNumber_eofContext _localctx = new ComplexNumber_eofContext(_ctx, getState());
		enterRule(_localctx, 16, RULE_complexNumber_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(110);
			((ComplexNumber_eofContext)_localctx).tmp = complexNumber();
			((ComplexNumber_eofContext)_localctx).ret =  ((ComplexNumber_eofContext)_localctx).tmp.ret;
			setState(112);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class ComplexNumberContext extends ParserRuleContext {
		public numberunit._ast.ASTComplexNumber ret =  null;
		public Token tmp0;
		public TerminalNode TComplexNumber() { return getToken(WeatherAntlrParser.TComplexNumber, 0); }
		public ComplexNumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_complexNumber; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterComplexNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitComplexNumber(this);
		}
	}

	public final ComplexNumberContext complexNumber() throws RecognitionException {
		ComplexNumberContext _localctx = new ComplexNumberContext(_ctx, getState());
		enterRule(_localctx, 18, RULE_complexNumber);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		numberunit._ast.ASTComplexNumber _aNode = null;
		_aNode=numberunit._ast.NumberUnitNodeFactory.createASTComplexNumber();
		((ComplexNumberContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(114);
			((ComplexNumberContext)_localctx).tmp0 = match(TComplexNumber);
			_aNode.setTComplexNumber(convertTComplexNumber(((ComplexNumberContext)_localctx).tmp0));
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Temperature_eofContext extends ParserRuleContext {
		public weather._ast.ASTTemperature ret =  null;
		public TemperatureContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public TemperatureContext temperature() {
			return getRuleContext(TemperatureContext.class,0);
		}
		public Temperature_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_temperature_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterTemperature_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitTemperature_eof(this);
		}
	}

	public final Temperature_eofContext temperature_eof() throws RecognitionException {
		Temperature_eofContext _localctx = new Temperature_eofContext(_ctx, getState());
		enterRule(_localctx, 20, RULE_temperature_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(117);
			((Temperature_eofContext)_localctx).tmp = temperature();
			((Temperature_eofContext)_localctx).ret =  ((Temperature_eofContext)_localctx).tmp.ret;
			setState(119);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class TemperatureContext extends ParserRuleContext {
		public weather._ast.ASTTemperature ret =  null;
		public Token tmp0;
		public TerminalNode TEMPERATURE() { return getToken(WeatherAntlrParser.TEMPERATURE, 0); }
		public TerminalNode TUnitNumber() { return getToken(WeatherAntlrParser.TUnitNumber, 0); }
		public TemperatureContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_temperature; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterTemperature(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitTemperature(this);
		}
	}

	public final TemperatureContext temperature() throws RecognitionException {
		TemperatureContext _localctx = new TemperatureContext(_ctx, getState());
		enterRule(_localctx, 22, RULE_temperature);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTTemperature _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTTemperature();
		((TemperatureContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(121);
			match(TEMPERATURE);
			}
			{
			setState(122);
			((TemperatureContext)_localctx).tmp0 = match(TUnitNumber);
			_aNode.setWeatherTemperature(convertTUnitNumber(((TemperatureContext)_localctx).tmp0));
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Humidity_eofContext extends ParserRuleContext {
		public weather._ast.ASTHumidity ret =  null;
		public HumidityContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public HumidityContext humidity() {
			return getRuleContext(HumidityContext.class,0);
		}
		public Humidity_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_humidity_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterHumidity_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitHumidity_eof(this);
		}
	}

	public final Humidity_eofContext humidity_eof() throws RecognitionException {
		Humidity_eofContext _localctx = new Humidity_eofContext(_ctx, getState());
		enterRule(_localctx, 24, RULE_humidity_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(125);
			((Humidity_eofContext)_localctx).tmp = humidity();
			((Humidity_eofContext)_localctx).ret =  ((Humidity_eofContext)_localctx).tmp.ret;
			setState(127);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class HumidityContext extends ParserRuleContext {
		public weather._ast.ASTHumidity ret =  null;
		public Token tmp0;
		public TerminalNode HUMIDITY() { return getToken(WeatherAntlrParser.HUMIDITY, 0); }
		public TerminalNode TUnitNumber() { return getToken(WeatherAntlrParser.TUnitNumber, 0); }
		public TerminalNode PERCENT() { return getToken(WeatherAntlrParser.PERCENT, 0); }
		public HumidityContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_humidity; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterHumidity(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitHumidity(this);
		}
	}

	public final HumidityContext humidity() throws RecognitionException {
		HumidityContext _localctx = new HumidityContext(_ctx, getState());
		enterRule(_localctx, 26, RULE_humidity);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTHumidity _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTHumidity();
		((HumidityContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(129);
			match(HUMIDITY);
			}
			{
			setState(130);
			((HumidityContext)_localctx).tmp0 = match(TUnitNumber);
			_aNode.setWeatherHumidity(convertTUnitNumber(((HumidityContext)_localctx).tmp0));
			}
			setState(135);
			_la = _input.LA(1);
			if (_la==PERCENT) {
				{
				setState(133);
				match(PERCENT);

				_aNode.setPERCENT(true);

				}
			}

			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Pressure_eofContext extends ParserRuleContext {
		public weather._ast.ASTPressure ret =  null;
		public PressureContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public PressureContext pressure() {
			return getRuleContext(PressureContext.class,0);
		}
		public Pressure_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_pressure_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterPressure_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitPressure_eof(this);
		}
	}

	public final Pressure_eofContext pressure_eof() throws RecognitionException {
		Pressure_eofContext _localctx = new Pressure_eofContext(_ctx, getState());
		enterRule(_localctx, 28, RULE_pressure_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(137);
			((Pressure_eofContext)_localctx).tmp = pressure();
			((Pressure_eofContext)_localctx).ret =  ((Pressure_eofContext)_localctx).tmp.ret;
			setState(139);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class PressureContext extends ParserRuleContext {
		public weather._ast.ASTPressure ret =  null;
		public Token tmp0;
		public TerminalNode PRESSURE() { return getToken(WeatherAntlrParser.PRESSURE, 0); }
		public TerminalNode TUnitNumber() { return getToken(WeatherAntlrParser.TUnitNumber, 0); }
		public PressureContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_pressure; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterPressure(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitPressure(this);
		}
	}

	public final PressureContext pressure() throws RecognitionException {
		PressureContext _localctx = new PressureContext(_ctx, getState());
		enterRule(_localctx, 30, RULE_pressure);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTPressure _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTPressure();
		((PressureContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(141);
			match(PRESSURE);
			}
			{
			setState(142);
			((PressureContext)_localctx).tmp0 = match(TUnitNumber);
			_aNode.setWeatherPressure(convertTUnitNumber(((PressureContext)_localctx).tmp0));
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Windstrength_eofContext extends ParserRuleContext {
		public weather._ast.ASTWindstrength ret =  null;
		public WindstrengthContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public WindstrengthContext windstrength() {
			return getRuleContext(WindstrengthContext.class,0);
		}
		public Windstrength_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_windstrength_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterWindstrength_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitWindstrength_eof(this);
		}
	}

	public final Windstrength_eofContext windstrength_eof() throws RecognitionException {
		Windstrength_eofContext _localctx = new Windstrength_eofContext(_ctx, getState());
		enterRule(_localctx, 32, RULE_windstrength_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(145);
			((Windstrength_eofContext)_localctx).tmp = windstrength();
			((Windstrength_eofContext)_localctx).ret =  ((Windstrength_eofContext)_localctx).tmp.ret;
			setState(147);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class WindstrengthContext extends ParserRuleContext {
		public weather._ast.ASTWindstrength ret =  null;
		public Token tmp0;
		public TerminalNode WINDSTRENGTH() { return getToken(WeatherAntlrParser.WINDSTRENGTH, 0); }
		public TerminalNode TUnitNumber() { return getToken(WeatherAntlrParser.TUnitNumber, 0); }
		public WindstrengthContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_windstrength; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterWindstrength(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitWindstrength(this);
		}
	}

	public final WindstrengthContext windstrength() throws RecognitionException {
		WindstrengthContext _localctx = new WindstrengthContext(_ctx, getState());
		enterRule(_localctx, 34, RULE_windstrength);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTWindstrength _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTWindstrength();
		((WindstrengthContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(149);
			match(WINDSTRENGTH);
			}
			{
			setState(150);
			((WindstrengthContext)_localctx).tmp0 = match(TUnitNumber);
			_aNode.setWeatherWindstrength(convertTUnitNumber(((WindstrengthContext)_localctx).tmp0));
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Winddirection_eofContext extends ParserRuleContext {
		public weather._ast.ASTWinddirection ret =  null;
		public WinddirectionContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public WinddirectionContext winddirection() {
			return getRuleContext(WinddirectionContext.class,0);
		}
		public Winddirection_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_winddirection_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterWinddirection_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitWinddirection_eof(this);
		}
	}

	public final Winddirection_eofContext winddirection_eof() throws RecognitionException {
		Winddirection_eofContext _localctx = new Winddirection_eofContext(_ctx, getState());
		enterRule(_localctx, 36, RULE_winddirection_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(153);
			((Winddirection_eofContext)_localctx).tmp = winddirection();
			((Winddirection_eofContext)_localctx).ret =  ((Winddirection_eofContext)_localctx).tmp.ret;
			setState(155);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class WinddirectionContext extends ParserRuleContext {
		public weather._ast.ASTWinddirection ret =  null;
		public Token tmp0;
		public TerminalNode WINDDIRECTION() { return getToken(WeatherAntlrParser.WINDDIRECTION, 0); }
		public TerminalNode TUnitNumber() { return getToken(WeatherAntlrParser.TUnitNumber, 0); }
		public WinddirectionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_winddirection; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterWinddirection(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitWinddirection(this);
		}
	}

	public final WinddirectionContext winddirection() throws RecognitionException {
		WinddirectionContext _localctx = new WinddirectionContext(_ctx, getState());
		enterRule(_localctx, 38, RULE_winddirection);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTWinddirection _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTWinddirection();
		((WinddirectionContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(157);
			match(WINDDIRECTION);
			}
			{
			setState(158);
			((WinddirectionContext)_localctx).tmp0 = match(TUnitNumber);
			_aNode.setWeatherWinddirection(convertTUnitNumber(((WinddirectionContext)_localctx).tmp0));
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Precipitationtype_eofContext extends ParserRuleContext {
		public weather._ast.ASTPrecipitationtype ret =  null;
		public PrecipitationtypeContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public PrecipitationtypeContext precipitationtype() {
			return getRuleContext(PrecipitationtypeContext.class,0);
		}
		public Precipitationtype_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_precipitationtype_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterPrecipitationtype_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitPrecipitationtype_eof(this);
		}
	}

	public final Precipitationtype_eofContext precipitationtype_eof() throws RecognitionException {
		Precipitationtype_eofContext _localctx = new Precipitationtype_eofContext(_ctx, getState());
		enterRule(_localctx, 40, RULE_precipitationtype_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(161);
			((Precipitationtype_eofContext)_localctx).tmp = precipitationtype();
			((Precipitationtype_eofContext)_localctx).ret =  ((Precipitationtype_eofContext)_localctx).tmp.ret;
			setState(163);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class PrecipitationtypeContext extends ParserRuleContext {
		public weather._ast.ASTPrecipitationtype ret =  null;
		public TerminalNode NONE() { return getToken(WeatherAntlrParser.NONE, 0); }
		public TerminalNode DRIZZLE() { return getToken(WeatherAntlrParser.DRIZZLE, 0); }
		public TerminalNode RAIN() { return getToken(WeatherAntlrParser.RAIN, 0); }
		public TerminalNode SNAIN() { return getToken(WeatherAntlrParser.SNAIN, 0); }
		public TerminalNode SNOW() { return getToken(WeatherAntlrParser.SNOW, 0); }
		public TerminalNode SLEET() { return getToken(WeatherAntlrParser.SLEET, 0); }
		public TerminalNode HAIL() { return getToken(WeatherAntlrParser.HAIL, 0); }
		public TerminalNode GRAUPEL() { return getToken(WeatherAntlrParser.GRAUPEL, 0); }
		public PrecipitationtypeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_precipitationtype; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterPrecipitationtype(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitPrecipitationtype(this);
		}
	}

	public final PrecipitationtypeContext precipitationtype() throws RecognitionException {
		PrecipitationtypeContext _localctx = new PrecipitationtypeContext(_ctx, getState());
		enterRule(_localctx, 42, RULE_precipitationtype);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTPrecipitationtype _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTPrecipitationtype();
		((PrecipitationtypeContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(165);
			match(T__0);
			}
			setState(196);
			switch (_input.LA(1)) {
			case NONE:
				{
				setState(166);
				match(NONE);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.NONE);

				}
				break;
			case DRIZZLE:
				{
				setState(168);
				match(DRIZZLE);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.DRIZZLE);

				}
				break;
			case RAIN:
				{
				setState(170);
				match(RAIN);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.RAIN);

				}
				break;
			case T__1:
				{
				setState(172);
				match(T__1);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.CONSTANT0);

				}
				break;
			case T__2:
				{
				setState(174);
				match(T__2);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.CONSTANT1);

				}
				break;
			case T__3:
				{
				setState(176);
				match(T__3);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.CONSTANT2);

				}
				break;
			case SNAIN:
				{
				setState(178);
				match(SNAIN);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.SNAIN);

				}
				break;
			case SNOW:
				{
				setState(180);
				match(SNOW);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.SNOW);

				}
				break;
			case T__4:
				{
				setState(182);
				match(T__4);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.CONSTANT3);

				}
				break;
			case T__5:
				{
				setState(184);
				match(T__5);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.CONSTANT4);

				}
				break;
			case SLEET:
				{
				setState(186);
				match(SLEET);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.SLEET);

				}
				break;
			case HAIL:
				{
				setState(188);
				match(HAIL);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.HAIL);

				}
				break;
			case T__6:
				{
				setState(190);
				match(T__6);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.CONSTANT5);

				}
				break;
			case GRAUPEL:
				{
				setState(192);
				match(GRAUPEL);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.GRAUPEL);

				}
				break;
			case T__7:
				{
				setState(194);
				match(T__7);

				_aNode.setPrecipitationType(weather._ast.ASTConstantsWeather.CONSTANT6);

				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Precipitationamount_eofContext extends ParserRuleContext {
		public weather._ast.ASTPrecipitationamount ret =  null;
		public PrecipitationamountContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public PrecipitationamountContext precipitationamount() {
			return getRuleContext(PrecipitationamountContext.class,0);
		}
		public Precipitationamount_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_precipitationamount_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterPrecipitationamount_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitPrecipitationamount_eof(this);
		}
	}

	public final Precipitationamount_eofContext precipitationamount_eof() throws RecognitionException {
		Precipitationamount_eofContext _localctx = new Precipitationamount_eofContext(_ctx, getState());
		enterRule(_localctx, 44, RULE_precipitationamount_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(198);
			((Precipitationamount_eofContext)_localctx).tmp = precipitationamount();
			((Precipitationamount_eofContext)_localctx).ret =  ((Precipitationamount_eofContext)_localctx).tmp.ret;
			setState(200);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class PrecipitationamountContext extends ParserRuleContext {
		public weather._ast.ASTPrecipitationamount ret =  null;
		public Token tmp0;
		public TerminalNode TUnitNumber() { return getToken(WeatherAntlrParser.TUnitNumber, 0); }
		public PrecipitationamountContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_precipitationamount; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterPrecipitationamount(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitPrecipitationamount(this);
		}
	}

	public final PrecipitationamountContext precipitationamount() throws RecognitionException {
		PrecipitationamountContext _localctx = new PrecipitationamountContext(_ctx, getState());
		enterRule(_localctx, 46, RULE_precipitationamount);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTPrecipitationamount _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTPrecipitationamount();
		((PrecipitationamountContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(202);
			match(T__8);
			}
			{
			setState(203);
			((PrecipitationamountContext)_localctx).tmp0 = match(TUnitNumber);
			_aNode.setWeatherPrecipitationamount(convertTUnitNumber(((PrecipitationamountContext)_localctx).tmp0));
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Clouding_eofContext extends ParserRuleContext {
		public weather._ast.ASTClouding ret =  null;
		public CloudingContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public CloudingContext clouding() {
			return getRuleContext(CloudingContext.class,0);
		}
		public Clouding_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_clouding_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterClouding_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitClouding_eof(this);
		}
	}

	public final Clouding_eofContext clouding_eof() throws RecognitionException {
		Clouding_eofContext _localctx = new Clouding_eofContext(_ctx, getState());
		enterRule(_localctx, 48, RULE_clouding_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(206);
			((Clouding_eofContext)_localctx).tmp = clouding();
			((Clouding_eofContext)_localctx).ret =  ((Clouding_eofContext)_localctx).tmp.ret;
			setState(208);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class CloudingContext extends ParserRuleContext {
		public weather._ast.ASTClouding ret =  null;
		public TerminalNode CLOUDING() { return getToken(WeatherAntlrParser.CLOUDING, 0); }
		public TerminalNode NONE() { return getToken(WeatherAntlrParser.NONE, 0); }
		public TerminalNode CIRROSTRATUS() { return getToken(WeatherAntlrParser.CIRROSTRATUS, 0); }
		public TerminalNode ALTOSTRATUS() { return getToken(WeatherAntlrParser.ALTOSTRATUS, 0); }
		public TerminalNode STRATUS() { return getToken(WeatherAntlrParser.STRATUS, 0); }
		public TerminalNode NIMBOSTRATUS() { return getToken(WeatherAntlrParser.NIMBOSTRATUS, 0); }
		public TerminalNode NOCTILUCENT() { return getToken(WeatherAntlrParser.NOCTILUCENT, 0); }
		public TerminalNode CIRRUS() { return getToken(WeatherAntlrParser.CIRRUS, 0); }
		public TerminalNode CIRROCUMULUS() { return getToken(WeatherAntlrParser.CIRROCUMULUS, 0); }
		public TerminalNode ALTOCUMULUS() { return getToken(WeatherAntlrParser.ALTOCUMULUS, 0); }
		public TerminalNode STRATOCUMULUS() { return getToken(WeatherAntlrParser.STRATOCUMULUS, 0); }
		public TerminalNode CUMULONIMBUS() { return getToken(WeatherAntlrParser.CUMULONIMBUS, 0); }
		public CloudingContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_clouding; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterClouding(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitClouding(this);
		}
	}

	public final CloudingContext clouding() throws RecognitionException {
		CloudingContext _localctx = new CloudingContext(_ctx, getState());
		enterRule(_localctx, 50, RULE_clouding);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTClouding _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTClouding();
		((CloudingContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(210);
			match(CLOUDING);
			}
			setState(241);
			switch (_input.LA(1)) {
			case NONE:
				{
				setState(211);
				match(NONE);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.NONE);

				}
				break;
			case CIRROSTRATUS:
				{
				setState(213);
				match(CIRROSTRATUS);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.CIRROSTRATUS);

				}
				break;
			case ALTOSTRATUS:
				{
				setState(215);
				match(ALTOSTRATUS);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.ALTOSTRATUS);

				}
				break;
			case STRATUS:
				{
				setState(217);
				match(STRATUS);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.STRATUS);

				}
				break;
			case NIMBOSTRATUS:
				{
				setState(219);
				match(NIMBOSTRATUS);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.NIMBOSTRATUS);

				}
				break;
			case NOCTILUCENT:
				{
				setState(221);
				match(NOCTILUCENT);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.NOCTILUCENT);

				}
				break;
			case T__9:
				{
				setState(223);
				match(T__9);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.CONSTANT7);

				}
				break;
			case CIRRUS:
				{
				setState(225);
				match(CIRRUS);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.CIRRUS);

				}
				break;
			case CIRROCUMULUS:
				{
				setState(227);
				match(CIRROCUMULUS);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.CIRROCUMULUS);

				}
				break;
			case ALTOCUMULUS:
				{
				setState(229);
				match(ALTOCUMULUS);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.ALTOCUMULUS);

				}
				break;
			case STRATOCUMULUS:
				{
				setState(231);
				match(STRATOCUMULUS);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.STRATOCUMULUS);

				}
				break;
			case T__10:
				{
				setState(233);
				match(T__10);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.CONSTANT8);

				}
				break;
			case T__11:
				{
				setState(235);
				match(T__11);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.CONSTANT9);

				}
				break;
			case T__12:
				{
				setState(237);
				match(T__12);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.CONSTANT10);

				}
				break;
			case CUMULONIMBUS:
				{
				setState(239);
				match(CUMULONIMBUS);

				_aNode.setCloudingType(weather._ast.ASTConstantsWeather.CUMULONIMBUS);

				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class Sight_eofContext extends ParserRuleContext {
		public weather._ast.ASTSight ret =  null;
		public SightContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public SightContext sight() {
			return getRuleContext(SightContext.class,0);
		}
		public Sight_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_sight_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterSight_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitSight_eof(this);
		}
	}

	public final Sight_eofContext sight_eof() throws RecognitionException {
		Sight_eofContext _localctx = new Sight_eofContext(_ctx, getState());
		enterRule(_localctx, 52, RULE_sight_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(243);
			((Sight_eofContext)_localctx).tmp = sight();
			((Sight_eofContext)_localctx).ret =  ((Sight_eofContext)_localctx).tmp.ret;
			setState(245);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class SightContext extends ParserRuleContext {
		public weather._ast.ASTSight ret =  null;
		public Token tmp0;
		public TerminalNode SIGHT() { return getToken(WeatherAntlrParser.SIGHT, 0); }
		public TerminalNode UNLIMITED() { return getToken(WeatherAntlrParser.UNLIMITED, 0); }
		public TerminalNode TUnitNumber() { return getToken(WeatherAntlrParser.TUnitNumber, 0); }
		public SightContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_sight; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterSight(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitSight(this);
		}
	}

	public final SightContext sight() throws RecognitionException {
		SightContext _localctx = new SightContext(_ctx, getState());
		enterRule(_localctx, 54, RULE_sight);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTSight _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTSight();
		((SightContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(247);
			match(SIGHT);
			}
			setState(252);
			switch (_input.LA(1)) {
			case TUnitNumber:
				{
				{
				setState(248);
				((SightContext)_localctx).tmp0 = match(TUnitNumber);
				_aNode.setWeatherSight(convertTUnitNumber(((SightContext)_localctx).tmp0));
				}
				}
				break;
			case UNLIMITED:
				{
				setState(250);
				match(UNLIMITED);

				_aNode.setUnlimited(true);

				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class WeatherPhenomena_eofContext extends ParserRuleContext {
		public weather._ast.ASTWeatherPhenomena ret =  null;
		public WeatherPhenomenaContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public WeatherPhenomenaContext weatherPhenomena() {
			return getRuleContext(WeatherPhenomenaContext.class,0);
		}
		public WeatherPhenomena_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_weatherPhenomena_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterWeatherPhenomena_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitWeatherPhenomena_eof(this);
		}
	}

	public final WeatherPhenomena_eofContext weatherPhenomena_eof() throws RecognitionException {
		WeatherPhenomena_eofContext _localctx = new WeatherPhenomena_eofContext(_ctx, getState());
		enterRule(_localctx, 56, RULE_weatherPhenomena_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(254);
			((WeatherPhenomena_eofContext)_localctx).tmp = weatherPhenomena();
			((WeatherPhenomena_eofContext)_localctx).ret =  ((WeatherPhenomena_eofContext)_localctx).tmp.ret;
			setState(256);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class WeatherPhenomenaContext extends ParserRuleContext {
		public weather._ast.ASTWeatherPhenomena ret =  null;
		public Token tmp0;
		public Token tmp1;
		public TerminalNode FOG() { return getToken(WeatherAntlrParser.FOG, 0); }
		public TerminalNode LANDSPOUT() { return getToken(WeatherAntlrParser.LANDSPOUT, 0); }
		public TerminalNode WATERSPOUT() { return getToken(WeatherAntlrParser.WATERSPOUT, 0); }
		public TerminalNode GUSTNADO() { return getToken(WeatherAntlrParser.GUSTNADO, 0); }
		public TerminalNode THUNDERSTORM() { return getToken(WeatherAntlrParser.THUNDERSTORM, 0); }
		public TerminalNode LPAREN() { return getToken(WeatherAntlrParser.LPAREN, 0); }
		public TerminalNode COMMA() { return getToken(WeatherAntlrParser.COMMA, 0); }
		public TerminalNode RPAREN() { return getToken(WeatherAntlrParser.RPAREN, 0); }
		public List<TerminalNode> TUnitNumber() { return getTokens(WeatherAntlrParser.TUnitNumber); }
		public TerminalNode TUnitNumber(int i) {
			return getToken(WeatherAntlrParser.TUnitNumber, i);
		}
		public WeatherPhenomenaContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_weatherPhenomena; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterWeatherPhenomena(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitWeatherPhenomena(this);
		}
	}

	public final WeatherPhenomenaContext weatherPhenomena() throws RecognitionException {
		WeatherPhenomenaContext _localctx = new WeatherPhenomenaContext(_ctx, getState());
		enterRule(_localctx, 58, RULE_weatherPhenomena);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTWeatherPhenomena _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTWeatherPhenomena();
		((WeatherPhenomenaContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			{
			setState(258);
			match(T__13);
			}
			setState(281);
			switch (_input.LA(1)) {
			case FOG:
				{
				setState(259);
				match(FOG);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.FOG);

				}
				break;
			case T__14:
				{
				setState(261);
				match(T__14);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONSTANT11);

				}
				break;
			case T__15:
				{
				setState(263);
				match(T__15);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONSTANT12);

				}
				break;
			case T__16:
				{
				setState(265);
				match(T__16);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONSTANT13);

				}
				break;
			case T__17:
				{
				setState(267);
				match(T__17);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONSTANT14);

				}
				break;
			case LANDSPOUT:
				{
				setState(269);
				match(LANDSPOUT);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.LANDSPOUT);

				}
				break;
			case WATERSPOUT:
				{
				setState(271);
				match(WATERSPOUT);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.WATERSPOUT);

				}
				break;
			case GUSTNADO:
				{
				setState(273);
				match(GUSTNADO);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.GUSTNADO);

				}
				break;
			case T__18:
				{
				setState(275);
				match(T__18);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONSTANT15);

				}
				break;
			case T__19:
				{
				setState(277);
				match(T__19);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONSTANT16);

				}
				break;
			case THUNDERSTORM:
				{
				setState(279);
				match(THUNDERSTORM);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.THUNDERSTORM);

				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			setState(292);
			_la = _input.LA(1);
			if (_la==LPAREN) {
				{
				{
				setState(283);
				match(LPAREN);
				}
				{
				{
				setState(284);
				((WeatherPhenomenaContext)_localctx).tmp0 = match(TUnitNumber);
				_aNode.setPosX(convertTUnitNumber(((WeatherPhenomenaContext)_localctx).tmp0));
				}
				}
				{
				setState(287);
				match(COMMA);
				}
				{
				{
				setState(288);
				((WeatherPhenomenaContext)_localctx).tmp1 = match(TUnitNumber);
				_aNode.setPosY(convertTUnitNumber(((WeatherPhenomenaContext)_localctx).tmp1));
				}
				}
				{
				setState(291);
				match(RPAREN);
				}
				}
			}

			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class OpticalPhenomena_eofContext extends ParserRuleContext {
		public weather._ast.ASTOpticalPhenomena ret =  null;
		public OpticalPhenomenaContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public OpticalPhenomenaContext opticalPhenomena() {
			return getRuleContext(OpticalPhenomenaContext.class,0);
		}
		public OpticalPhenomena_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_opticalPhenomena_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterOpticalPhenomena_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitOpticalPhenomena_eof(this);
		}
	}

	public final OpticalPhenomena_eofContext opticalPhenomena_eof() throws RecognitionException {
		OpticalPhenomena_eofContext _localctx = new OpticalPhenomena_eofContext(_ctx, getState());
		enterRule(_localctx, 60, RULE_opticalPhenomena_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(294);
			((OpticalPhenomena_eofContext)_localctx).tmp = opticalPhenomena();
			((OpticalPhenomena_eofContext)_localctx).ret =  ((OpticalPhenomena_eofContext)_localctx).tmp.ret;
			setState(296);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class OpticalPhenomenaContext extends ParserRuleContext {
		public weather._ast.ASTOpticalPhenomena ret =  null;
		public TerminalNode RAINBOW() { return getToken(WeatherAntlrParser.RAINBOW, 0); }
		public TerminalNode MIRAGE() { return getToken(WeatherAntlrParser.MIRAGE, 0); }
		public OpticalPhenomenaContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_opticalPhenomena; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterOpticalPhenomena(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitOpticalPhenomena(this);
		}
	}

	public final OpticalPhenomenaContext opticalPhenomena() throws RecognitionException {
		OpticalPhenomenaContext _localctx = new OpticalPhenomenaContext(_ctx, getState());
		enterRule(_localctx, 62, RULE_opticalPhenomena);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTOpticalPhenomena _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTOpticalPhenomena();
		((OpticalPhenomenaContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			{
			setState(298);
			match(T__20);
			}
			setState(313);
			switch (_input.LA(1)) {
			case RAINBOW:
				{
				setState(299);
				match(RAINBOW);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.RAINBOW);

				}
				break;
			case T__21:
				{
				setState(301);
				match(T__21);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONSTANT17);

				}
				break;
			case T__22:
				{
				setState(303);
				match(T__22);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONSTANT18);

				}
				break;
			case T__23:
				{
				setState(305);
				match(T__23);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONSTANT19);

				}
				break;
			case T__24:
				{
				setState(307);
				match(T__24);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONSTANT20);

				}
				break;
			case MIRAGE:
				{
				setState(309);
				match(MIRAGE);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.MIRAGE);

				}
				break;
			case T__25:
				{
				setState(311);
				match(T__25);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONSTANT21);

				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class ArtificialPhenomena_eofContext extends ParserRuleContext {
		public weather._ast.ASTArtificialPhenomena ret =  null;
		public ArtificialPhenomenaContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public ArtificialPhenomenaContext artificialPhenomena() {
			return getRuleContext(ArtificialPhenomenaContext.class,0);
		}
		public ArtificialPhenomena_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_artificialPhenomena_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterArtificialPhenomena_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitArtificialPhenomena_eof(this);
		}
	}

	public final ArtificialPhenomena_eofContext artificialPhenomena_eof() throws RecognitionException {
		ArtificialPhenomena_eofContext _localctx = new ArtificialPhenomena_eofContext(_ctx, getState());
		enterRule(_localctx, 64, RULE_artificialPhenomena_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(315);
			((ArtificialPhenomena_eofContext)_localctx).tmp = artificialPhenomena();
			((ArtificialPhenomena_eofContext)_localctx).ret =  ((ArtificialPhenomena_eofContext)_localctx).tmp.ret;
			setState(317);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class ArtificialPhenomenaContext extends ParserRuleContext {
		public weather._ast.ASTArtificialPhenomena ret =  null;
		public TerminalNode CONTRAILS() { return getToken(WeatherAntlrParser.CONTRAILS, 0); }
		public TerminalNode SMOG() { return getToken(WeatherAntlrParser.SMOG, 0); }
		public ArtificialPhenomenaContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_artificialPhenomena; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterArtificialPhenomena(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitArtificialPhenomena(this);
		}
	}

	public final ArtificialPhenomenaContext artificialPhenomena() throws RecognitionException {
		ArtificialPhenomenaContext _localctx = new ArtificialPhenomenaContext(_ctx, getState());
		enterRule(_localctx, 66, RULE_artificialPhenomena);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTArtificialPhenomena _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTArtificialPhenomena();
		((ArtificialPhenomenaContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			{
			setState(319);
			match(T__26);
			}
			setState(326);
			switch (_input.LA(1)) {
			case CONTRAILS:
				{
				setState(320);
				match(CONTRAILS);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONTRAILS);

				}
				break;
			case SMOG:
				{
				setState(322);
				match(SMOG);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.SMOG);

				}
				break;
			case T__27:
				{
				setState(324);
				match(T__27);

				_aNode.setPhenomenaType(weather._ast.ASTConstantsWeather.CONSTANT22);

				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static final String _serializedATN =
		"\3\u0430\ud6d1\u8206\uad2d\u4417\uaef1\u8d80\uaadd\3N\u014b\4\2\t\2\4"+
		"\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4\13\t"+
		"\13\4\f\t\f\4\r\t\r\4\16\t\16\4\17\t\17\4\20\t\20\4\21\t\21\4\22\t\22"+
		"\4\23\t\23\4\24\t\24\4\25\t\25\4\26\t\26\4\27\t\27\4\30\t\30\4\31\t\31"+
		"\4\32\t\32\4\33\t\33\4\34\t\34\4\35\t\35\4\36\t\36\4\37\t\37\4 \t \4!"+
		"\t!\4\"\t\"\4#\t#\3\2\3\2\3\2\3\2\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3"+
		"\3\3\3\3\3\3\5\3W\n\3\3\4\3\4\3\4\3\4\3\5\3\5\3\5\3\6\3\6\3\6\3\6\3\7"+
		"\3\7\3\7\3\b\3\b\3\b\3\b\3\t\3\t\3\t\3\t\5\to\n\t\3\n\3\n\3\n\3\n\3\13"+
		"\3\13\3\13\3\f\3\f\3\f\3\f\3\r\3\r\3\r\3\r\3\16\3\16\3\16\3\16\3\17\3"+
		"\17\3\17\3\17\3\17\3\17\5\17\u008a\n\17\3\20\3\20\3\20\3\20\3\21\3\21"+
		"\3\21\3\21\3\22\3\22\3\22\3\22\3\23\3\23\3\23\3\23\3\24\3\24\3\24\3\24"+
		"\3\25\3\25\3\25\3\25\3\26\3\26\3\26\3\26\3\27\3\27\3\27\3\27\3\27\3\27"+
		"\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27"+
		"\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\5\27\u00c7\n\27"+
		"\3\30\3\30\3\30\3\30\3\31\3\31\3\31\3\31\3\32\3\32\3\32\3\32\3\33\3\33"+
		"\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33"+
		"\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33"+
		"\3\33\5\33\u00f4\n\33\3\34\3\34\3\34\3\34\3\35\3\35\3\35\3\35\3\35\5\35"+
		"\u00ff\n\35\3\36\3\36\3\36\3\36\3\37\3\37\3\37\3\37\3\37\3\37\3\37\3\37"+
		"\3\37\3\37\3\37\3\37\3\37\3\37\3\37\3\37\3\37\3\37\3\37\3\37\3\37\3\37"+
		"\3\37\5\37\u011c\n\37\3\37\3\37\3\37\3\37\3\37\3\37\3\37\3\37\3\37\5\37"+
		"\u0127\n\37\3 \3 \3 \3 \3!\3!\3!\3!\3!\3!\3!\3!\3!\3!\3!\3!\3!\3!\3!\5"+
		"!\u013c\n!\3\"\3\"\3\"\3\"\3#\3#\3#\3#\3#\3#\3#\5#\u0149\n#\3#\2\2$\2"+
		"\4\6\b\n\f\16\20\22\24\26\30\32\34\36 \"$&(*,.\60\62\64\668:<>@BD\2\2"+
		"\u015d\2F\3\2\2\2\4V\3\2\2\2\6X\3\2\2\2\b\\\3\2\2\2\n_\3\2\2\2\fc\3\2"+
		"\2\2\16f\3\2\2\2\20n\3\2\2\2\22p\3\2\2\2\24t\3\2\2\2\26w\3\2\2\2\30{\3"+
		"\2\2\2\32\177\3\2\2\2\34\u0083\3\2\2\2\36\u008b\3\2\2\2 \u008f\3\2\2\2"+
		"\"\u0093\3\2\2\2$\u0097\3\2\2\2&\u009b\3\2\2\2(\u009f\3\2\2\2*\u00a3\3"+
		"\2\2\2,\u00a7\3\2\2\2.\u00c8\3\2\2\2\60\u00cc\3\2\2\2\62\u00d0\3\2\2\2"+
		"\64\u00d4\3\2\2\2\66\u00f5\3\2\2\28\u00f9\3\2\2\2:\u0100\3\2\2\2<\u0104"+
		"\3\2\2\2>\u0128\3\2\2\2@\u012c\3\2\2\2B\u013d\3\2\2\2D\u0141\3\2\2\2F"+
		"G\5\4\3\2GH\b\2\1\2HI\7\2\2\3I\3\3\2\2\2JK\5\b\5\2KL\b\3\1\2LW\3\2\2\2"+
		"MN\5\f\7\2NO\b\3\1\2OW\3\2\2\2PQ\5\24\13\2QR\b\3\1\2RW\3\2\2\2ST\5\20"+
		"\t\2TU\b\3\1\2UW\3\2\2\2VJ\3\2\2\2VM\3\2\2\2VP\3\2\2\2VS\3\2\2\2W\5\3"+
		"\2\2\2XY\5\b\5\2YZ\b\4\1\2Z[\7\2\2\3[\7\3\2\2\2\\]\7F\2\2]^\b\5\1\2^\t"+
		"\3\2\2\2_`\5\f\7\2`a\b\6\1\2ab\7\2\2\3b\13\3\2\2\2cd\7G\2\2de\b\7\1\2"+
		"e\r\3\2\2\2fg\5\20\t\2gh\b\b\1\2hi\7\2\2\3i\17\3\2\2\2jk\7H\2\2ko\b\t"+
		"\1\2lm\7I\2\2mo\b\t\1\2nj\3\2\2\2nl\3\2\2\2o\21\3\2\2\2pq\5\24\13\2qr"+
		"\b\n\1\2rs\7\2\2\3s\23\3\2\2\2tu\7J\2\2uv\b\13\1\2v\25\3\2\2\2wx\5\30"+
		"\r\2xy\b\f\1\2yz\7\2\2\3z\27\3\2\2\2{|\7*\2\2|}\7H\2\2}~\b\r\1\2~\31\3"+
		"\2\2\2\177\u0080\5\34\17\2\u0080\u0081\b\16\1\2\u0081\u0082\7\2\2\3\u0082"+
		"\33\3\2\2\2\u0083\u0084\7,\2\2\u0084\u0085\7H\2\2\u0085\u0086\b\17\1\2"+
		"\u0086\u0089\3\2\2\2\u0087\u0088\7\63\2\2\u0088\u008a\b\17\1\2\u0089\u0087"+
		"\3\2\2\2\u0089\u008a\3\2\2\2\u008a\35\3\2\2\2\u008b\u008c\5 \21\2\u008c"+
		"\u008d\b\20\1\2\u008d\u008e\7\2\2\3\u008e\37\3\2\2\2\u008f\u0090\79\2"+
		"\2\u0090\u0091\7H\2\2\u0091\u0092\b\21\1\2\u0092!\3\2\2\2\u0093\u0094"+
		"\5$\23\2\u0094\u0095\b\22\1\2\u0095\u0096\7\2\2\3\u0096#\3\2\2\2\u0097"+
		"\u0098\7B\2\2\u0098\u0099\7H\2\2\u0099\u009a\b\23\1\2\u009a%\3\2\2\2\u009b"+
		"\u009c\5(\25\2\u009c\u009d\b\24\1\2\u009d\u009e\7\2\2\3\u009e\'\3\2\2"+
		"\2\u009f\u00a0\7=\2\2\u00a0\u00a1\7H\2\2\u00a1\u00a2\b\25\1\2\u00a2)\3"+
		"\2\2\2\u00a3\u00a4\5,\27\2\u00a4\u00a5\b\26\1\2\u00a5\u00a6\7\2\2\3\u00a6"+
		"+\3\2\2\2\u00a7\u00c6\7\3\2\2\u00a8\u00a9\7$\2\2\u00a9\u00c7\b\27\1\2"+
		"\u00aa\u00ab\7:\2\2\u00ab\u00c7\b\27\1\2\u00ac\u00ad\7\61\2\2\u00ad\u00c7"+
		"\b\27\1\2\u00ae\u00af\7\4\2\2\u00af\u00c7\b\27\1\2\u00b0\u00b1\7\5\2\2"+
		"\u00b1\u00c7\b\27\1\2\u00b2\u00b3\7\6\2\2\u00b3\u00c7\b\27\1\2\u00b4\u00b5"+
		"\7 \2\2\u00b5\u00c7\b\27\1\2\u00b6\u00b7\7A\2\2\u00b7\u00c7\b\27\1\2\u00b8"+
		"\u00b9\7\7\2\2\u00b9\u00c7\b\27\1\2\u00ba\u00bb\7\b\2\2\u00bb\u00c7\b"+
		"\27\1\2\u00bc\u00bd\7.\2\2\u00bd\u00c7\b\27\1\2\u00be\u00bf\7;\2\2\u00bf"+
		"\u00c7\b\27\1\2\u00c0\u00c1\7\t\2\2\u00c1\u00c7\b\27\1\2\u00c2\u00c3\7"+
		"\60\2\2\u00c3\u00c7\b\27\1\2\u00c4\u00c5\7\n\2\2\u00c5\u00c7\b\27\1\2"+
		"\u00c6\u00a8\3\2\2\2\u00c6\u00aa\3\2\2\2\u00c6\u00ac\3\2\2\2\u00c6\u00ae"+
		"\3\2\2\2\u00c6\u00b0\3\2\2\2\u00c6\u00b2\3\2\2\2\u00c6\u00b4\3\2\2\2\u00c6"+
		"\u00b6\3\2\2\2\u00c6\u00b8\3\2\2\2\u00c6\u00ba\3\2\2\2\u00c6\u00bc\3\2"+
		"\2\2\u00c6\u00be\3\2\2\2\u00c6\u00c0\3\2\2\2\u00c6\u00c2\3\2\2\2\u00c6"+
		"\u00c4\3\2\2\2\u00c7-\3\2\2\2\u00c8\u00c9\5\60\31\2\u00c9\u00ca\b\30\1"+
		"\2\u00ca\u00cb\7\2\2\3\u00cb/\3\2\2\2\u00cc\u00cd\7\13\2\2\u00cd\u00ce"+
		"\7H\2\2\u00ce\u00cf\b\31\1\2\u00cf\61\3\2\2\2\u00d0\u00d1\5\64\33\2\u00d1"+
		"\u00d2\b\32\1\2\u00d2\u00d3\7\2\2\3\u00d3\63\3\2\2\2\u00d4\u00f3\7@\2"+
		"\2\u00d5\u00d6\7$\2\2\u00d6\u00f4\b\33\1\2\u00d7\u00d8\7C\2\2\u00d8\u00f4"+
		"\b\33\1\2\u00d9\u00da\7!\2\2\u00da\u00f4\b\33\1\2\u00db\u00dc\7\'\2\2"+
		"\u00dc\u00f4\b\33\1\2\u00dd\u00de\7%\2\2\u00de\u00f4\b\33\1\2\u00df\u00e0"+
		"\7/\2\2\u00e0\u00f4\b\33\1\2\u00e1\u00e2\7\f\2\2\u00e2\u00f4\b\33\1\2"+
		"\u00e3\u00e4\7\65\2\2\u00e4\u00f4\b\33\1\2\u00e5\u00e6\7>\2\2\u00e6\u00f4"+
		"\b\33\1\2\u00e7\u00e8\7\62\2\2\u00e8\u00f4\b\33\1\2\u00e9\u00ea\7(\2\2"+
		"\u00ea\u00f4\b\33\1\2\u00eb\u00ec\7\r\2\2\u00ec\u00f4\b\33\1\2\u00ed\u00ee"+
		"\7\16\2\2\u00ee\u00f4\b\33\1\2\u00ef\u00f0\7\17\2\2\u00f0\u00f4\b\33\1"+
		"\2\u00f1\u00f2\7&\2\2\u00f2\u00f4\b\33\1\2\u00f3\u00d5\3\2\2\2\u00f3\u00d7"+
		"\3\2\2\2\u00f3\u00d9\3\2\2\2\u00f3\u00db\3\2\2\2\u00f3\u00dd\3\2\2\2\u00f3"+
		"\u00df\3\2\2\2\u00f3\u00e1\3\2\2\2\u00f3\u00e3\3\2\2\2\u00f3\u00e5\3\2"+
		"\2\2\u00f3\u00e7\3\2\2\2\u00f3\u00e9\3\2\2\2\u00f3\u00eb\3\2\2\2\u00f3"+
		"\u00ed\3\2\2\2\u00f3\u00ef\3\2\2\2\u00f3\u00f1\3\2\2\2\u00f4\65\3\2\2"+
		"\2\u00f5\u00f6\58\35\2\u00f6\u00f7\b\34\1\2\u00f7\u00f8\7\2\2\3\u00f8"+
		"\67\3\2\2\2\u00f9\u00fe\7+\2\2\u00fa\u00fb\7H\2\2\u00fb\u00ff\b\35\1\2"+
		"\u00fc\u00fd\7\64\2\2\u00fd\u00ff\b\35\1\2\u00fe\u00fa\3\2\2\2\u00fe\u00fc"+
		"\3\2\2\2\u00ff9\3\2\2\2\u0100\u0101\5<\37\2\u0101\u0102\b\36\1\2\u0102"+
		"\u0103\7\2\2\3\u0103;\3\2\2\2\u0104\u011b\7\20\2\2\u0105\u0106\7E\2\2"+
		"\u0106\u011c\b\37\1\2\u0107\u0108\7\21\2\2\u0108\u011c\b\37\1\2\u0109"+
		"\u010a\7\22\2\2\u010a\u011c\b\37\1\2\u010b\u010c\7\23\2\2\u010c\u011c"+
		"\b\37\1\2\u010d\u010e\7\24\2\2\u010e\u011c\b\37\1\2\u010f\u0110\7#\2\2"+
		"\u0110\u011c\b\37\1\2\u0111\u0112\7-\2\2\u0112\u011c\b\37\1\2\u0113\u0114"+
		"\7\66\2\2\u0114\u011c\b\37\1\2\u0115\u0116\7\25\2\2\u0116\u011c\b\37\1"+
		"\2\u0117\u0118\7\26\2\2\u0118\u011c\b\37\1\2\u0119\u011a\7D\2\2\u011a"+
		"\u011c\b\37\1\2\u011b\u0105\3\2\2\2\u011b\u0107\3\2\2\2\u011b\u0109\3"+
		"\2\2\2\u011b\u010b\3\2\2\2\u011b\u010d\3\2\2\2\u011b\u010f\3\2\2\2\u011b"+
		"\u0111\3\2\2\2\u011b\u0113\3\2\2\2\u011b\u0115\3\2\2\2\u011b\u0117\3\2"+
		"\2\2\u011b\u0119\3\2\2\2\u011c\u0126\3\2\2\2\u011d\u011e\7\67\2\2\u011e"+
		"\u011f\7H\2\2\u011f\u0120\b\37\1\2\u0120\u0121\3\2\2\2\u0121\u0122\7<"+
		"\2\2\u0122\u0123\7H\2\2\u0123\u0124\b\37\1\2\u0124\u0125\3\2\2\2\u0125"+
		"\u0127\78\2\2\u0126\u011d\3\2\2\2\u0126\u0127\3\2\2\2\u0127=\3\2\2\2\u0128"+
		"\u0129\5@!\2\u0129\u012a\b \1\2\u012a\u012b\7\2\2\3\u012b?\3\2\2\2\u012c"+
		"\u013b\7\27\2\2\u012d\u012e\7?\2\2\u012e\u013c\b!\1\2\u012f\u0130\7\30"+
		"\2\2\u0130\u013c\b!\1\2\u0131\u0132\7\31\2\2\u0132\u013c\b!\1\2\u0133"+
		"\u0134\7\32\2\2\u0134\u013c\b!\1\2\u0135\u0136\7\33\2\2\u0136\u013c\b"+
		"!\1\2\u0137\u0138\7)\2\2\u0138\u013c\b!\1\2\u0139\u013a\7\34\2\2\u013a"+
		"\u013c\b!\1\2\u013b\u012d\3\2\2\2\u013b\u012f\3\2\2\2\u013b\u0131\3\2"+
		"\2\2\u013b\u0133\3\2\2\2\u013b\u0135\3\2\2\2\u013b\u0137\3\2\2\2\u013b"+
		"\u0139\3\2\2\2\u013cA\3\2\2\2\u013d\u013e\5D#\2\u013e\u013f\b\"\1\2\u013f"+
		"\u0140\7\2\2\3\u0140C\3\2\2\2\u0141\u0148\7\35\2\2\u0142\u0143\7\37\2"+
		"\2\u0143\u0149\b#\1\2\u0144\u0145\7\"\2\2\u0145\u0149\b#\1\2\u0146\u0147"+
		"\7\36\2\2\u0147\u0149\b#\1\2\u0148\u0142\3\2\2\2\u0148\u0144\3\2\2\2\u0148"+
		"\u0146\3\2\2\2\u0149E\3\2\2\2\fVn\u0089\u00c6\u00f3\u00fe\u011b\u0126"+
		"\u013b\u0148";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}