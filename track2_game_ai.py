EMPTY = 0
BLUE = 1
YELLOW = 2

LINES = (
    (0, 1, 2), (3, 4, 5), (6, 7, 8),
    (0, 3, 6), (1, 4, 7), (2, 5, 8),
    (0, 4, 8), (2, 4, 6),
)

CANDIDATE_ORDER = (4, 0, 2, 6, 8, 1, 3, 5, 7)

NEIGHBORS = (
    (1, 3, 4),
    (0, 2, 4),
    (1, 4, 5),
    (0, 4, 6),
    (0, 1, 2, 3, 5, 6, 7, 8),
    (2, 4, 8),
    (3, 4, 7),
    (4, 6, 8),
    (4, 5, 7),
)


class Move:
    def __init__(self, color, dst, src=None, kind="place"):
        self.color = color
        self.src = src
        self.dst = dst
        self.kind = kind

    def is_place(self):
        return self.src is None

    def to_cells(self):
        return {
            "kind": self.kind,
            "color": color_name(self.color),
            "src": None if self.src is None else self.src + 1,
            "dst": self.dst + 1,
        }

    def __repr__(self):
        data = self.to_cells()
        if data["src"] is None:
            return "Move(%s -> %s, %s)" % (data["color"], data["dst"], data["kind"])
        return "Move(%s: %s -> %s, %s)" % (
            data["color"], data["src"], data["dst"], data["kind"]
        )


def other(color):
    if color == BLUE:
        return YELLOW
    if color == YELLOW:
        return BLUE
    raise ValueError("unknown color: %r" % (color,))


def color_name(color):
    if color == BLUE:
        return "blue"
    if color == YELLOW:
        return "yellow"
    if color == EMPTY:
        return "empty"
    return "unknown"


def normalize_board(board):
    if isinstance(board, str):
        return parse_board(board)
    if len(board) != 9:
        raise ValueError("board must contain 9 cells")
    normalized = []
    for cell in board:
        if cell in (EMPTY, BLUE, YELLOW):
            normalized.append(cell)
        elif cell in (".", "_", "0", None):
            normalized.append(EMPTY)
        elif cell in ("B", "b", "blue", "BLUE"):
            normalized.append(BLUE)
        elif cell in ("Y", "y", "yellow", "YELLOW"):
            normalized.append(YELLOW)
        else:
            raise ValueError("unknown board cell: %r" % (cell,))
    return tuple(normalized)


def parse_board(text):
    mapping = {
        ".": EMPTY, "_": EMPTY, "0": EMPTY,
        "B": BLUE, "b": BLUE, "1": BLUE,
        "Y": YELLOW, "y": YELLOW, "2": YELLOW,
    }
    cells = []
    for char in text:
        if char.isspace():
            continue
        if char not in mapping:
            raise ValueError("unknown board char: %r" % char)
        cells.append(mapping[char])
    if len(cells) != 9:
        raise ValueError("board text must describe 9 cells")
    return tuple(cells)


def board_to_text(board):
    board = normalize_board(board)
    chars = {EMPTY: ".", BLUE: "B", YELLOW: "Y"}
    return "".join(chars[cell] for cell in board)


def count_color(board, color):
    return normalize_board(board).count(color)


def winner(board):
    board = normalize_board(board)
    for a, b, c in LINES:
        if board[a] != EMPTY and board[a] == board[b] == board[c]:
            return board[a]
    return EMPTY


def is_full_placement_done(board):
    board = normalize_board(board)
    return count_color(board, BLUE) == 3 and count_color(board, YELLOW) == 3


def phase_name(board):
    return "move" if is_full_placement_done(board) else "place"


def legal_moves(board, color, adjacent_only=False):
    board = normalize_board(board)
    empty_cells = [idx for idx in CANDIDATE_ORDER if board[idx] == EMPTY]
    if winner(board):
        return []

    if not is_full_placement_done(board):
        if count_color(board, color) >= 3:
            return []
        return [Move(color=color, dst=dst, kind="place") for dst in empty_cells]

    moves = []
    for src in CANDIDATE_ORDER:
        if board[src] != color:
            continue
        if adjacent_only:
            targets = [dst for dst in NEIGHBORS[src] if board[dst] == EMPTY]
        else:
            targets = empty_cells
        for dst in targets:
            if dst != src:
                moves.append(Move(color=color, src=src, dst=dst, kind="move"))
    return moves


def apply_move(board, move):
    board = list(normalize_board(board))
    if move.dst < 0 or move.dst >= 9:
        raise ValueError("destination out of range")
    if board[move.dst] != EMPTY:
        raise ValueError("destination is not empty")

    if move.src is None:
        if count_color(board, move.color) >= 3:
            raise ValueError("a team can only place 3 physical pieces")
        board[move.dst] = move.color
    else:
        if move.src < 0 or move.src >= 9:
            raise ValueError("source out of range")
        if move.src == move.dst:
            raise ValueError("source and destination must be different")
        if board[move.src] != move.color:
            raise ValueError("source is not the moving team's piece")
        board[move.src] = EMPTY
        board[move.dst] = move.color
    return tuple(board)


def infer_placement_turn(board, first_player):
    board = normalize_board(board)
    first_count = count_color(board, first_player)
    second_count = count_color(board, other(first_player))
    if is_full_placement_done(board):
        return other(first_player)
    if first_count == second_count:
        return first_player
    if first_count == second_count + 1:
        return other(first_player)
    raise ValueError("board is not a legal alternating placement state")


def transition_move(previous, observed, actor_color, adjacent_only=False):
    previous = normalize_board(previous)
    observed = normalize_board(observed)
    for move in legal_moves(previous, actor_color, adjacent_only=adjacent_only):
        if apply_move(previous, move) == observed:
            return move
    return None


def detect_tamper(previous, observed, protected_color):
    previous = normalize_board(previous)
    observed = normalize_board(observed)
    lost = []
    gained = []
    for idx in range(9):
        if previous[idx] == protected_color and observed[idx] != protected_color:
            lost.append(idx)
        if previous[idx] != protected_color and observed[idx] == protected_color:
            gained.append(idx)

    if len(lost) == 1 and len(gained) == 1:
        return Move(
            color=protected_color,
            src=gained[0],
            dst=lost[0],
            kind="restore",
        )
    return None


class Track2AI:
    def __init__(self, ai_color=BLUE, first_player=BLUE, max_depth=8, adjacent_only=False):
        self.ai_color = ai_color
        self.first_player = first_player
        self.max_depth = max_depth
        self.adjacent_only = adjacent_only

    def choose_move(self, board, to_move=None):
        board = normalize_board(board)
        if to_move is None:
            to_move = self.ai_color
        moves = legal_moves(board, to_move, adjacent_only=self.adjacent_only)
        if not moves:
            return None

        win_now = self._first_winning_move(board, to_move, moves)
        if win_now is not None:
            return win_now

        block = self._first_blocking_move(board, to_move, moves)
        if block is not None:
            return block

        best_move = None
        if to_move == self.ai_color:
            best_score = -1000000
            for move in moves:
                score = self._minimax(
                    apply_move(board, move),
                    other(to_move),
                    self.max_depth - 1,
                    -1000000,
                    1000000,
                    set(),
                )
                if score > best_score:
                    best_score = score
                    best_move = move
        else:
            best_score = 1000000
            for move in moves:
                score = self._minimax(
                    apply_move(board, move),
                    other(to_move),
                    self.max_depth - 1,
                    -1000000,
                    1000000,
                    set(),
                )
                if score < best_score:
                    best_score = score
                    best_move = move
        return best_move

    def _first_winning_move(self, board, color, moves):
        for move in moves:
            if winner(apply_move(board, move)) == color:
                return move
        return None

    def _first_blocking_move(self, board, color, moves):
        opponent = other(color)
        opponent_winning_dsts = set()
        for opp_move in legal_moves(board, opponent, adjacent_only=self.adjacent_only):
            if winner(apply_move(board, opp_move)) == opponent:
                opponent_winning_dsts.add(opp_move.dst)
        for move in moves:
            if move.dst in opponent_winning_dsts:
                return move
        return None

    def _minimax(self, board, to_move, depth, alpha, beta, seen):
        win = winner(board)
        if win == self.ai_color:
            return 10000 + depth
        if win == other(self.ai_color):
            return -10000 - depth
        if depth <= 0:
            return self._evaluate(board)

        key = (board, to_move)
        if key in seen:
            return self._evaluate(board) - 2
        seen.add(key)

        moves = legal_moves(board, to_move, adjacent_only=self.adjacent_only)
        if not moves:
            seen.remove(key)
            return self._evaluate(board)

        if to_move == self.ai_color:
            value = -1000000
            for move in moves:
                score = self._minimax(
                    apply_move(board, move),
                    other(to_move),
                    depth - 1,
                    alpha,
                    beta,
                    seen,
                )
                if score > value:
                    value = score
                if value > alpha:
                    alpha = value
                if alpha >= beta:
                    break
        else:
            value = 1000000
            for move in moves:
                score = self._minimax(
                    apply_move(board, move),
                    other(to_move),
                    depth - 1,
                    alpha,
                    beta,
                    seen,
                )
                if score < value:
                    value = score
                if value < beta:
                    beta = value
                if alpha >= beta:
                    break

        seen.remove(key)
        return value

    def _evaluate(self, board):
        board = normalize_board(board)
        opponent = other(self.ai_color)
        score = 0

        if board[4] == self.ai_color:
            score += 8
        elif board[4] == opponent:
            score -= 8

        for idx in (0, 2, 6, 8):
            if board[idx] == self.ai_color:
                score += 3
            elif board[idx] == opponent:
                score -= 3

        for line in LINES:
            ai_count = 0
            opponent_count = 0
            for idx in line:
                if board[idx] == self.ai_color:
                    ai_count += 1
                elif board[idx] == opponent:
                    opponent_count += 1
            if opponent_count == 0:
                score += (0, 4, 60, 10000)[ai_count]
            if ai_count == 0:
                score -= (0, 4, 70, 10000)[opponent_count]

        return score


def _self_test():
    ai = Track2AI(ai_color=BLUE, first_player=BLUE, max_depth=8)

    move = ai.choose_move(parse_board("BB......."), to_move=BLUE)
    assert move.dst == 2, move

    move = ai.choose_move(parse_board("YY......."), to_move=BLUE)
    assert move.dst == 2, move

    move = ai.choose_move(parse_board("BYYYBB..."), to_move=BLUE)
    assert move.src == 5 and move.dst == 8, move

    previous = parse_board("BYYYBB...")
    observed = parse_board("BYYY.B..B")
    correction = detect_tamper(previous, observed, BLUE)
    assert correction.src == 8 and correction.dst == 4, correction

    print("track2_game_ai self-test passed")


if __name__ == "__main__":
    _self_test()
