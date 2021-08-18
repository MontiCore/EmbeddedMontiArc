package de.monticore.lang.gdl;

import de.monticore.lang.gdl._cocos.*;
import de.monticore.lang.gdl._ast.*;
import de.se_rwth.commons.logging.Log;

public class ASTGameExpressionCoCo implements GDLASTGameExpressionCoCo {
	@Override
	public void check(ASTGameExpression node) {
		if (node.getType() instanceof ASTGameRole) {
			if(node.getArgumentsList().size() != 1) {
				Log.error("GameRole definition got too much arguments");
			}
			// only on root
			// if(!(node.getArguments(0) instanceof ASTGameValue)) {
			// Log.error("GameRole definition argument must be of type GameValue");
			// }
		}

		if (node.getType() instanceof ASTGameInit) {
			if(node.getArgumentsList().size() != 1) {
				Log.error("GameInit definition got too much arguments");
			}
			if(node.getArgumentsList().size() >= 1 && !(node.getArguments(0) instanceof ASTGameExpression)) {
				Log.error("GameInit definition argument must be of type GameExpression");
			}
			if(node.getArgumentsList().size() >= 1 && !( ((ASTGameExpression)node.getArguments(0)).getType() instanceof ASTGameFunction)) {
				Log.error("First argument of the GameInit argument tuple must be of type GameFunction");
			}
		}

		if (node.getType() instanceof ASTGameTerminal) {
			for (int i = 0; i < node.getArgumentsList().size(); i++) {
				if(!(node.getArguments(i) instanceof ASTGameExpression)) {
					Log.error("Arguments of GameTerminal must be of type GameExpression");
				}
			}
		}

		if (node.getType() instanceof ASTGameInference) {
			if(node.getArgumentsList().size() < 2) {
				Log.error("GameInference definitions must have at least two arguments of type GameExpression");
			}
			for (int i = 0; i < node.getArgumentsList().size(); i++) {
				if(!(node.getArguments(i) instanceof ASTGameExpression)) {
					Log.error("Arguments of GameInference must be of type GameExpression");
				}
			}
		}

		if (node.getType() instanceof ASTGameNext) {
			if(node.getArgumentsList().size() != 1) {
				Log.error("GameNext definition got too much arguments");
			}
			if(node.getArgumentsList().size() >= 1 && !(node.getArguments(0) instanceof ASTGameExpression)) {
				Log.error("GameNext definition argument must be of type GameExpression");
			}
			if(node.getArgumentsList().size() >= 1 && (node.getArguments(0) instanceof ASTGameExpression) && !( ((ASTGameExpression)node.getArguments(0)).getType() instanceof ASTGameFunction)) {
				Log.error("First argument of the GameNext argument tuple must be of type GameFunction");
			}
		}

		if (node.getType() instanceof ASTGameTrue) {
			if(node.getArgumentsList().size() != 1) {
				Log.error("GameTrue definition got too much arguments");
			}
			if(node.getArgumentsList().size() >= 1 && !(node.getArguments(0) instanceof ASTGameExpression)) {
				Log.error("GameTrue definition argument must be of type GameExpression");
			}
			if(node.getArgumentsList().size() >= 1 && !( ((ASTGameExpression)node.getArguments(0)).getType() instanceof ASTGameFunction)) {
				Log.error("First argument of the GameTrue argument tuple must be of type GameFunction");
			}
		}

		if (node.getType() instanceof ASTGameLegal) {
			if(node.getArgumentsList().size() != 2) {
				Log.error("GameLegal definition must have exactly two arguments");
			}
			if(node.getArgumentsList().size() >= 1 && !(node.getArguments(0) instanceof ASTGameToken)) {
				Log.error("The first GameLegal definition argument must be of type GameToken");
			}
			if(node.getArgumentsList().size() >= 2 && !(node.getArguments(1) instanceof ASTGameExpression)) {
				Log.error("The second GameLegal definition argument must be of type GameExpression");
			}
			if(node.getArgumentsList().size() >= 2 && (node.getArguments(1) instanceof ASTGameExpression) && !( ((ASTGameExpression)node.getArguments(1)).getType() instanceof ASTGameFunction)) {
				Log.error("First argument of the GameLegal argument tuple must be of type GameFunction");
			}
		}

		if(node.getType() instanceof ASTGameDoes) {
			if(node.getArgumentsList().size() != 2) {
				Log.error("GameDoes definition must have exactly two arguments");
			}
			if(node.getArgumentsList().size() >= 1 && !(node.getArguments(0) instanceof ASTGameToken || node.getArguments(0) instanceof ASTGameValue)) {
				Log.error("The first GameDoes definition argument must be of type GameToken or GameValue");
			}
			if(node.getArgumentsList().size() >= 2 && !(node.getArguments(1) instanceof ASTGameExpression)) {
				Log.error("The second GameDoes definition argument must be of type GameExpression");
			}
			if(node.getArgumentsList().size() >= 2 && !(((ASTGameExpression)node.getArguments(1)).getType() instanceof ASTGameFunction)) {
				Log.error("First argument of the GameDoes argument tuple must be of type GameFunction");
			}
		}

		if(node.getType() instanceof ASTGameNot) {
			if(node.getArgumentsList().size() != 1) {
				Log.error("GameNot definition must have exactly one argument");
			}
			if(node.getArgumentsList().size() >= 1 && !(node.getArguments(0) instanceof ASTGameFunction || node.getArguments(0) instanceof ASTGameExpression)) {
				Log.error("GameNot argument must be of type GameFunction or GameExpression");
			}
			// if(node.getArgumentsList().size() >= 1 && node.getArguments(0) instanceof ASTGameExpression && !(((ASTGameExpression)node.getArguments(0)).getType() instanceof ASTGameFunction)) {
			// 	Log.error("First argument of the GameNot argument tuple must be of type GameFunction");
			// }
			// if(node.getArguments(0) instanceof ASTGameExpression && !(((ASTGameExpression)node.getArguments(0)).getType() instanceof ASTGameFunction)) {
			// 	Log.error("First argument of the GameNot argument tuple must be of type GameFunction");
			// }
		}

		if (node.getType() instanceof ASTGameDistinct) {
			if(node.getArgumentsList().size() != 2) {
				Log.error("GameDistinct definition must have exactly two arguments");
			}
			if(node.getArgumentsList().size() >= 2 && !(node.getArguments(0) instanceof ASTGameToken || node.getArguments(0) instanceof ASTGameValue && node.getArguments(1) instanceof ASTGameToken || node.getArguments(1) instanceof ASTGameValue)) {
				Log.error("GameDistinct arguments must be of type GameToken or GameValue");
			}
		}

		if (node.getType() instanceof ASTGameGoal) {
			if(node.getArgumentsList().size() != 2) {
				Log.error("GameGoal definition must have exactly two arguments");
			}
			if(node.getArgumentsList().size() >= 2 && !(node.getArguments(0) instanceof ASTGameToken && node.getArguments(1) instanceof ASTGameValue)) {
				Log.error("GameGoal arguments must be of type GameValue");
			}
		}

		if (node.getType() instanceof ASTGameFunction) {
			for (int i = 0; i < node.getArgumentsList().size(); i++) {
				if(!(node.getArguments(i) instanceof ASTGameToken || node.getArguments(i) instanceof ASTGameValue )) {
					Log.error("Arguments of GameFunction must be of type GameValue or GameToken");
				}
			}
		}

	}
}