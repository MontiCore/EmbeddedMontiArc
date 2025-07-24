import chess.pgn
import io

# Install chess (pip install chess)

# ------------------------------------------------
# ------------------------------------------------
# ---------------- PASTE PGN HERE ----------------
# ------------------------------------------------
# ------------------------------------------------

pgn = """1.e4 e5 2.Nf3 Nc6 3.Bb5 a6 4.Ba4 Nf6 5.O-O Be7 6.Re1 b5 7.Bb3
d6 8.c3 O-O 9.h3 h6 10.d4 Re8 11.Nbd2 Bf8 12.Nf1 Bd7 13.Ng3
Na5 14.Bc2 c5 15.b3 Nc6 16.d5 Ne7 17.Be3 Ng6 18.Qd2 Nh7 19.a4
Nh4 20.Nxh4 Qxh4 21.Qe2 Qd8 22.b4 Qc7 23.Rec1 c4 24.Ra3 Rec8
25.Rca1 Qd8 26.f4 Nf6 27.fxe5 dxe5 28.Qf1 Ne8 29.Qf2 Nd6
30.Bb6 Qe8 31.R3a2 Be7 32.Bc5 Bf8 33.Nf5 Bxf5 34.exf5 f6
35.Bxd6 Bxd6 36.axb5 axb5 37.Be4 Rxa2 38.Qxa2 Qd7 39.Qa7 Rc7
40.Qb6 Rb7 41.Ra8+ Kf7 42.Qa6 Qc7 43.Qc6 Qb6+ 44.Kf1 Rb8
45.Ra6 1-0"""



[PAWN, KNIGHT, BISHOP, ROOK, QUEEN, KING] = range(1, 7)
PIECE_NAMES = [None, "pawn", "knight", "bishop", "rook", "queen", "king"]
FILE_NAMES = ["a", "b", "c", "d", "e", "f", "g", "h"]
RANK_NAMES = ["1", "2", "3", "4", "5", "6", "7", "8"]

def get_player_prefix(w):
    return "white" if w else "black"

def print_move(w, piece, start_col, start_row, dest_col, dest_row):
    print("{} (move {} {} {} {} {})".format(get_player_prefix(w), piece, start_col, start_row, dest_col, dest_row))

def print_field(piece, col, row):
    print("(field {} {} {})".format(col, row, piece))

def print_castle_rights(b_w, side, allowed):
    allowed_str = "allowed" if allowed else "disallowed"
    print("(castle {} {} {})".format(b_w, side, allowed_str))

def print_control(w):
    print("(control {})".format(get_player_prefix(w)))

def print_en_passant(square):
    col = "none" if square is None else FILE_NAMES[chess.square_file(square)]
    print("(enPassant {})".format(col))

def print_state(board):
    for i in range(0, 64):
        col = FILE_NAMES[chess.square_file(i)]
        row = RANK_NAMES[chess.square_rank(i)]
        if board.piece_type_at(i) is None:
            # blank
            print_field("blank", col, row)
        else:
            print_field(str(get_player_prefix(board.color_at(i)) + "_" + PIECE_NAMES[board.piece_type_at(i)]), col, row)

    print_control(board.turn)

    print_castle_rights("white", "short", board.has_kingside_castling_rights(True))
    print_castle_rights("white", "long", board.has_queenside_castling_rights(True))
    print_castle_rights("black", "short", board.has_kingside_castling_rights(False))
    print_castle_rights("black", "long", board.has_queenside_castling_rights(False))

    print_en_passant(board.ep_square)

pgn = io.StringIO(pgn)
first_game = chess.pgn.read_game(pgn)
board = first_game.board()

promotion_warnings = ""

state_delimiter = "STATE"
command_delimiter = "COMMAND"

print(state_delimiter)
print_state(board)

for move in first_game.mainline_moves():

    if move.promotion is not None:
        promotion_warnings = promotion_warnings + "WARNING: Promotion detected: {}".format(PIECE_NAMES[move.promotion]) + "\n"

    from_col = FILE_NAMES[chess.square_file(move.from_square)]
    from_row = RANK_NAMES[chess.square_rank(move.from_square)]

    to_col = FILE_NAMES[chess.square_file(move.to_square)]
    to_row = RANK_NAMES[chess.square_rank(move.to_square)]

    print(command_delimiter)

    print_move(board.color_at(move.from_square) ,str(get_player_prefix(board.color_at(move.from_square)) + "_" + PIECE_NAMES[board.piece_type_at(move.from_square)]), from_col, from_row, to_col, to_row)

    board.push(move)

    print(state_delimiter)
    print_state(board)


print(promotion_warnings)


